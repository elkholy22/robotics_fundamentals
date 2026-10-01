#include "rclcpp/rclcpp.hpp"

#include "map_and_messages/msg/pose.hpp"
#include "red_fundamentals/srv/move_to_position.hpp"
#include "create_fundamentals/srv/store_song.hpp"
#include "create_fundamentals/srv/play_song.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <optional>
#include <cctype>
#include <cmath>
#include <algorithm>

using Pose           = map_and_messages::msg::Pose;
using MoveToPosition = red_fundamentals::srv::MoveToPosition;
using StoreSong      = create_fundamentals::srv::StoreSong;
using PlaySong       = create_fundamentals::srv::PlaySong;

using std::placeholders::_1;
using namespace std::chrono_literals;

const int  SONG_GOLD   = 2;
const int  SONG_PICKUP = 3;

// wait 5 seconds while playing the song at the gold/ pickup cell.
const auto STAY_DURATION = 5s;

// ---- a maze cell ----
struct Cell {
    int row;
    int col;
};

rclcpp::Node::SharedPtr                      node;
rclcpp::Client<MoveToPosition>::SharedPtr    move_client_;
rclcpp::Client<StoreSong>::SharedPtr         store_song_client_;
rclcpp::Client<PlaySong>::SharedPtr          play_song_client_;
rclcpp::Subscription<Pose>::SharedPtr        pose_sub_;

std::mutex            pose_mutex_;
std::optional<Pose>   latest_pose_;
std::vector<Cell>     gold_cells_;
std::vector<Cell>     pickup_cells_;

void pose_callback(const Pose &msg)
{
    std::lock_guard<std::mutex> lk(pose_mutex_);
    latest_pose_ = msg;
}

std::optional<Pose> get_pose()
{
    std::lock_guard<std::mutex> lk(pose_mutex_);
    return latest_pose_;
}

void store_song(int number, std::vector<uint32_t> song)
{
    auto req    = std::make_shared<StoreSong::Request>();
    req->number = number;
    req->song   = song;
    store_song_client_->async_send_request(req);
}

void play_song(int number)
{
    auto req    = std::make_shared<PlaySong::Request>();
    req->number = number;
    play_song_client_->async_send_request(req);
}

std::vector<Cell> parse_cells(const std::string &path)
{
    std::vector<Cell> cells;

    std::ifstream f(path);
    if (!f.is_open()) {
        printf("Error: could not open '%s'\n", path.c_str());
        return cells;
    }

    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();

    // blank out everything that isn't a digit, then read the numbers
    for (char &ch : text)
        if (!isdigit(ch)) ch = ' ';

    std::vector<int> nums;
    std::stringstream nss(text);
    int v;
    while (nss >> v)
        nums.push_back(v);

    // each pair of numbers is one cell: row, column
    for (size_t k = 0; k + 1 < nums.size(); k += 2)
        cells.push_back({nums[k], nums[k + 1]});

    return cells;
}
int manhattan(const Cell &a, const Cell &b)
{
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

// Drive to one cell. Blocks until the planner finishes or fails. Retries a
// few times for transient failures, then gives up so the mission never hangs.
bool drive_to(const Cell &target)
{
    auto p = get_pose();
    if (p.has_value() && p->row == target.row && p->column == target.col) {
        printf("Already at (row=%d, col=%d), no move needed.\n", target.row, target.col);
        return true;
    }

    const int MAX_TRIES = 3;
    for (int attempt = 1; attempt <= MAX_TRIES && rclcpp::ok(); attempt++) {
        printf("Driving to (row=%d, col=%d)  [attempt %d/%d]...\n",
               target.row, target.col, attempt, MAX_TRIES);

        if (!move_client_->wait_for_service(std::chrono::seconds(2))) {
            printf("move_to_position service not available.\n");
            std::this_thread::sleep_for(500ms);
            continue;
        }

        auto req    = std::make_shared<MoveToPosition::Request>();
        req->row    = target.row;
        req->column = target.col;

        auto future = move_client_->async_send_request(req);

        if (future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
            printf("move_to_position timed out.\n");
            std::this_thread::sleep_for(500ms);
            continue;
        }

        bool ok = false;
        try {
            ok = future.get()->success;
        } catch (const std::exception &e) {
            printf("move_to_position threw: %s\n", e.what());
            ok = false;
        }

        if (ok) {
            printf("Reached (row=%d, col=%d).\n", target.row, target.col);
            return true;
        }
        printf("move_to_position failed for (row=%d, col=%d).\n", target.row, target.col);
        std::this_thread::sleep_for(500ms);
    }
    printf("GIVING UP on (row=%d, col=%d).\n", target.row, target.col);
    return false;
}
void stay_and_sing(int song, const char *label)
{
    printf("%s: staying still and playing song %d for %lld s...\n",
           label, song, (long long)std::chrono::duration_cast<std::chrono::seconds>(STAY_DURATION).count());
    play_song(song);
    std::this_thread::sleep_for(STAY_DURATION);   // no motion commands -> robot is still
}


void run_mission()
{

    store_song_client_->wait_for_service();
    store_song(SONG_GOLD,   {72,40, 76,40, 79,40, 84,40, 79,40, 76,40, 72,40, 84,40}); 
    store_song(SONG_PICKUP, {60,40, 64,40, 67,40, 72,40, 76,40, 72,40, 67,40, 60,40}); 

    // wait until localized 
    printf("Waiting for localization (first /pose)...\n");
    while (rclcpp::ok() && !get_pose().has_value())
        std::this_thread::sleep_for(200ms);
    if (!rclcpp::ok()) return;
    printf("Localized. Starting the challenge.\n");

    //wait for the planner service to be up
    printf("Waiting for /move_to_position service...\n");
    while (rclcpp::ok() && !move_client_->wait_for_service(1s))
        printf("  ...still waiting for /move_to_position\n");
    if (!rclcpp::ok()) return;

    // current cell = where we localized
    Cell cur;
    {
        auto p = get_pose().value();
        cur = {p.row, p.column};
    }
    printf("Start cell: (row=%d, col=%d)\n", cur.row, cur.col);

    // collect all gold, nearest first ,greedy by manhattan distance
    std::vector<Cell> remaining = gold_cells_;
    int collected = 0;
    while (!remaining.empty() && rclcpp::ok()) {
        int best = 0;
        for (size_t k = 1; k < remaining.size(); k++)
            if (manhattan(cur, remaining[k]) < manhattan(cur, remaining[best]))
                best = (int)k;
        Cell target = remaining[best];

        if (drive_to(target)) {
            stay_and_sing(SONG_GOLD, "Collecting gold");
            collected++;
            cur = target;     
        } else {
            printf("Skipping gold (row=%d, col=%d) - unreachable.\n", target.row, target.col);
        }
        remaining.erase(remaining.begin() + best);
    }
    printf("Collected %d / %zu gold.\n", collected, gold_cells_.size());

    //leave the maze via the nearest reachable pickup
    if (!pickup_cells_.empty() && rclcpp::ok()) {
        std::vector<Cell> pickups = pickup_cells_;
        std::sort(pickups.begin(), pickups.end(),
                  [&](const Cell &a, const Cell &b) {
                      return manhattan(cur, a) < manhattan(cur, b);
                  });

        bool left = false;
        for (const auto &pk : pickups) {
            if (drive_to(pk)) {
                stay_and_sing(SONG_PICKUP, "At pickup");
                cur  = pk;
                left = true;
                break;
            }
            printf("Pickup (row=%d, col=%d) unreachable, trying next...\n", pk.row, pk.col);
        }

        if (left) printf("CHALLENGE COMPLETE: gold collected and left the maze.\n");
        else      printf("FAILED to reach any pickup location.\n");
    } else {
        printf("No pickup locations given - cannot leave the maze.\n");
    }

    printf("Mission finished.\n");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("challenge");

    std::string gold_path   = (argc > 1) ? argv[1] : "gold.txt";
    std::string pickup_path = (argc > 2) ? argv[2] : "pickup.txt";
    printf("gold file:   %s\n", gold_path.c_str());
    printf("pickup file: %s\n", pickup_path.c_str());

    gold_cells_   = parse_cells(gold_path);
    pickup_cells_ = parse_cells(pickup_path);

    printf("Parsed %zu gold cells:",   gold_cells_.size());
    for (auto &c : gold_cells_)   printf(" (r%d,c%d)", c.row, c.col);
    printf("\nParsed %zu pickup cells:", pickup_cells_.size());
    for (auto &c : pickup_cells_) printf(" (r%d,c%d)", c.row, c.col);
    printf("\n");

    move_client_       = node->create_client<MoveToPosition>("move_to_position");
    store_song_client_ = node->create_client<StoreSong>("store_song");
    play_song_client_  = node->create_client<PlaySong>("play_song");
    pose_sub_          = node->create_subscription<Pose>("pose", 10, std::bind(&pose_callback, _1));

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    run_mission();

    executor.cancel();      
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
