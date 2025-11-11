#include "ecat/task.hpp"
#include "../power.hpp"
#include "../io.hpp"
#include "ecat/s2s_func.hpp"
#include "../CtrlContext.hpp"
#include "Joint.hpp"
#include "armPosiSender.hpp"
#include "../udp_tool.hpp"
#include "../wrist_IK_FK.hpp"

#include <fmt/ranges.h>
#include <iostream>
#include <qiuniu/init.h>
#include <spdlog/spdlog.h>
#include <vector>
#include <boost/program_options.hpp>
#include <signal.h>
#include <iterator>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/bool.hpp"

#include <sys/mman.h>        // mlockall
#include <sched.h>           // CPU_* macros
#include <unistd.h>          // close
#include <csignal>           
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>


struct program_input
{
  io_data *io;
  int count = 0;
  void init() {}
  void operator()() {
    count ++;
    if (!(count % 1000))
    {
      // uint8_t val = *io->io_address;
      uint16_t val = (static_cast<int>(*io->io_address) >> io->io_bit_pos) & 0x01;
      // uint8_t val = (*(uint8_t*)io->io_address >> io->io_bit_pos) & 0x01;
      // std::cout << "read slave: " << io->slave_pos << " io: " << std::hex << io->io_id << " value is " << val << "\n";
    }
    count %= 10000;
  }
};

struct program_output
{
  io_data *io;
  int count = 0;
  void init() {}
  void operator()() {
    count ++;
    if (count == 10000) {
      // *(uint16_t*)io->io_address |= (1 << io->io_bit_pos);
      *io->io_address = 0xff;
      // std::cout << "write slave: " << io->slave_pos << " count: " << count << " io: " << std::hex << io->io_id << " value is " << ((static_cast<int>(*io->io_address) >> io->io_bit_pos) & 0x01) << "\n";
    }
    else if (count == 5000) {
      // *(uint16_t*)io->io_address |= (1 << io->io_bit_pos);
      *io->io_address = 0x00;
      // std::cout << "write slave: " << io->slave_pos << " count: " << count << " io: " << std::hex << io->io_id << " value is " << ((static_cast<int>(*io->io_address) >> io->io_bit_pos) & 0x01) << "\n";
    }
    // std::cout << "write slave: " << io->slave_pos << " io: " << io->io_id << " count: " << count << " value is " << static_cast<int>(*io->io_address)<< "\n";
    count %= 10000;
  }
};

static int axis_count = 0;
static std::vector<std::unique_ptr<axis_data>> axes;
static std::vector<std::function<void ()>> programs;
static std::vector<std::unique_ptr<Joint>> joints;
static CtrlContext ctx;
static double t_k_1 = 0;

static void usage(const char * program_name)
{
  printf(" example:\n"
         "         %s \n"
         "         %s -h\n"
         "         %s -f eni.xml\n"
         "         %s -c 500000\n",
         program_name, program_name, program_name, program_name);
}

ecat::task *tt;
// cleanup routine
void shutdownAllJoints() {
  spdlog::info("Shutdown ALL Joints...");
  for (auto& item : joints) {
      item->disablePower();  
  }
}

void signal_hander(int sig){
  // shutdownAllJoints();
  signal(SIGINT, SIG_DFL);
  tt->break_();
}

// ==== your existing headers/types ====
// #include "whatever_defines_joint_axis_ctx.hpp"
// extern std::vector<std::unique_ptr<Joint>> joints;
// extern std::vector<std::unique_ptr<axis_data>> axes;
// extern ctx_t ctx;   // or whatever your context is
// void shutdownAllJoints();
// int init_broadcast_socket(sockaddr_in* addr);
// std::pair<double,double> solveFK(...);
// using ecat::task, ecat::dc_mode, etc.
// =====================================

class TeleopNode : public rclcpp::Node {
public:
  TeleopNode() : rclcpp::Node("teleop")
  {
    /////////////////////////////////////////////////////////////////////////////
    ////////////////// Start Declaring Publisher and Subscriber /////////////////
    /////////////////////////////////////////////////////////////////////////////
    
    // publish current joint state every 10ms (Be aware that ROS2 timer is not accurate)
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), [this]() { this->publish_joint_states(); });

    // subscribe target joint position
    target_joint_posi_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_command_pub", 1,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        rclcpp::Time current_stamp = msg->header.stamp;
        if (last_stamp_.nanoseconds() > 0) {

          // compute time delta
          rclcpp::Duration delta_duration = current_stamp - last_stamp_;
          double dt = delta_duration.seconds(); double min_dt = 0.008;
          if (dt < min_dt) dt = min_dt;

          std::scoped_lock lk(ctx.current_m, ctx.target_m);
          // store target q from msg
          for (size_t i = 0; i < num_joints-2; ++i) {
            ctx.target_q_rot[i+2] = msg->position[i];
            if (is_rot(i+2)) ctx.target_q[i+2] = msg->position[i];
          }

          // rot to linear conversion
          auto [lf, lb] = solveIK(ctx.target_q_rot[7],  ctx.target_q_rot[8], Side::Left);
          ctx.target_q[7]  = lf; ctx.target_q[8]  = lb;
          auto [rf, rb] = solveIK(ctx.target_q_rot[14], ctx.target_q_rot[15], Side::Right);
          ctx.target_q[14] = rf; ctx.target_q[15] = rb;

          // // update target velocity
          // for (int i = 0; i < num_joints; ++i) {
          //   if (ctx.enable_flags[i]) {
          //     ctx.target_q_dot[i] = (ctx.target_q[i] - ctx.q[i]) / dt;
          //     ctx.target_q_dot[i] = (ctx.target_q[i] - ctx.old_target_q[i]) / dt;
          //     ctx.target_q_dot[i] = std::clamp(ctx.target_q_dot[i], -100.0f, 100.0f);
          //     ctx.old_target_q[i] = ctx.target_q[i];
          //   } else {
          //     ctx.target_q_dot[i] = 0.0f;
          //   }
          // }
          
        }
        last_stamp_ = current_stamp;
      });
    
    // enable/disable subscriber
    joint_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "enable", 1,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        ctx.enable_flags = {
            false, false, // head
            msg->data, msg->data, msg->data, msg->data, msg->data, false, false, // left arm
            false, msg->data, msg->data, msg->data, msg->data, false, false  // right arm
        };
        ctx.q_err.fill(0.0);
        ctx.command_q_dot.fill(0.0);
      });
    
    /////////////////////////////////////////////////////////////////////////////
    ////////////////// End Declaring Publisher and Subscriber ///////////////////
    /////////////////////////////////////////////////////////////////////////////

    // --- Parameters (replace boost::program_options) ---
    file_name_ = this->declare_parameter<std::string>("file_name", "");
    cycle_time_ = this->declare_parameter<int64_t>("cycle_time", 1'000'000);
    shift_time_ = this->declare_parameter<int64_t>("shift_time", 300'000);
    log_level_  = this->declare_parameter<int>("log_level", 0);
    priority_   = this->declare_parameter<int>("priority", 90);
    affinity_   = this->declare_parameter<int>("affinity", 1);
    op_mode_    = this->declare_parameter<int>("op_mode", 9);
    master_id_  = this->declare_parameter<int>("master_id", 0);

    // set initial joint mode
    std::array<int, num_joints> init_mode = {
      9, 9, // head
      9, 9, 9, 9, 9, 9, 9, // left arm
      9, 9, 9, 9, 9, 9, 9  // right arm
    };
    ctx.joint_mode = init_mode;

    spdlog::set_level(static_cast<spdlog::level::level_enum>(log_level_));

    // --- Your one-time init that used to be at the top of main() ---
    joints.reserve(16);
    qiuniu_init();

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
      RCLCPP_FATAL(get_logger(), "mlockall failed: %s", strerror(errno));
      throw std::runtime_error("mlockall failed");
    }

    // --- UDP broadcast socket (same as before) ---
    sockfd_ = init_broadcast_socket(&broadcast_addr_);
    if (sockfd_ < 0) {
      throw std::runtime_error("init_broadcast_socket failed");
    }

    // --- EtherCAT task setup (same as before but as members) ---
    task_ = std::make_unique<ecat::task>(master_id_);
    task_->priority(priority_);

    // CPU affinity
    cpu_set_t cpus; CPU_ZERO(&cpus); CPU_SET(affinity_, &cpus);
    task_->cpu_affinity(&cpus, sizeof(cpus));

    task_->record(false);
    // ESI mode (from your code)
    RCLCPP_INFO(get_logger(), "Use ESI XML");
    task_->cycle_time(cycle_time_, shift_time_);
    task_->dc_mode(ecat::dc_mode::master_follow_slave);

    // --- CONFIG CALLBACK (paste your existing mapping body) ---
    task_->set_config_callback([this] {
      std::uint16_t slave_count = task_->slave_count();

      for (std::uint16_t slave_pos = 0; slave_pos < slave_count; ++slave_pos) {
        auto profile_no = task_->profile_no(slave_pos);
        if (profile_no == 402) {
          int n_axis_in_slave = 1, slot_pos;
          int slots_count = task_->slots_count(slave_pos);
          int slots_index_increment = task_->slot_index_increment(slave_pos);
          if (slots_count > 0) {
            assert(slots_index_increment != -1);
            n_axis_in_slave = slots_count;
          }
          assert(!(slots_count < 0));

          for (slot_pos = 0; slot_pos < n_axis_in_slave; ++slot_pos) {
            const int index_offset = slot_pos * slots_index_increment;

            auto axis = std::make_unique<axis_data>();
            axis->slave_pos = slave_pos;
            axis->axis_id   = axis_count++;               // your global
            axis->joint_id  = ctx.joint_map[axis->axis_id];
            axis->joint_name= ctx.joint_name[axis->joint_id];


            task_->try_register_pdo_entry(axis->control_word, slave_pos, {static_cast<ecat::pdo_index_type>(0x6040 + index_offset), 0});
            task_->try_register_pdo_entry(axis->target_position, slave_pos, {static_cast<ecat::pdo_index_type>(0x607a + index_offset), 0});
            task_->try_register_pdo_entry(axis->target_velocity, slave_pos, {static_cast<ecat::pdo_index_type>(0x60ff + index_offset), 0});
            task_->try_register_pdo_entry(axis->target_torque,   slave_pos, {static_cast<ecat::pdo_index_type>(0x6071 + index_offset), 0});
            task_->try_register_pdo_entry(axis->mode_of_operation, slave_pos, {static_cast<ecat::pdo_index_type>(0x6060 + index_offset), 0});

            task_->try_register_pdo_entry(axis->status_word,            slave_pos, {static_cast<ecat::pdo_index_type>(0x6041 + index_offset), 0});
            task_->try_register_pdo_entry(axis->position_actual_value,  slave_pos, {static_cast<ecat::pdo_index_type>(0x6064 + index_offset), 0});
            task_->try_register_pdo_entry(axis->velocity_actual_value,  slave_pos, {static_cast<ecat::pdo_index_type>(0x606c + index_offset), 0});
            task_->try_register_pdo_entry(axis->torque_actual_value,    slave_pos, {static_cast<ecat::pdo_index_type>(0x6077 + index_offset), 0});
            task_->try_register_pdo_entry(axis->mode_of_operation_display, slave_pos, {static_cast<ecat::pdo_index_type>(0x6061 + index_offset), 0});
            task_->try_register_pdo_entry(axis->error_code,             slave_pos, {static_cast<ecat::pdo_index_type>(0x603f + index_offset), 0});

            axis->mode = ctx.joint_mode[axis->joint_id];

            // Keep your safe order for Joint construction
            axis_data* axis_ptr = axis.get();
            joints.push_back(std::make_unique<Joint>(*axis_ptr, ctx));
            axes.push_back(std::move(axis));
          }
        }
      }

      ecat::S2SConfig::use().count();
    });

    // --- RECEIVE CALLBACK (same as before) ---
    task_->set_receive_callback([this] {
      std::scoped_lock lk(ctx.current_m);

      for (auto &j : joints) { (*j)(); }

      // linear to rot calculation (unchanged)
      auto [roll, pitch] = solveFK(ctx.q_rot[7],  ctx.q_rot[8],  ctx.q[7],  ctx.q[8],  Side::Left);
      ctx.q_rot[7]  = roll; ctx.q_rot[8]  = pitch;
      std::tie(roll, pitch) = solveFK(ctx.q_rot[14], ctx.q_rot[15], ctx.q[14], ctx.q[15], Side::Right);
      ctx.q_rot[14] = roll; ctx.q_rot[15] = pitch;

      ctx.main_cnt = (ctx.main_cnt + 1) % 100000;
      if (ctx.main_cnt % 100 == 0) {
        sendto(sockfd_, static_cast<const void*>(&ctx.q_rot[0]), ctx.q_rot.size() * sizeof(ctx.q_rot[0]), 0,
               (struct sockaddr *)&broadcast_addr_, sizeof(broadcast_addr_));
        // spdlog::info("q = [{}]", fmt::join(std::begin(ctx.q_rot), std::end(ctx.q_rot), ", "));
      }
    });

    // --- Start EtherCAT; wait in a background thread so ROS can spin ---
    task_->record(false);
    task_->start();
    waiter_ = std::thread([this]{
      task_->wait();   // blocks until release/stop
    });
  }

  void stop() {
    static std::atomic<bool> done{false};
    if (done.exchange(true)) return;  // make idempotent

    if (task_) {
      task_->break_();               // 1. signal loop to stop
    }
    if (waiter_.joinable()) {
      waiter_.join();                // 2. wait for task_->wait() to return
    }
    if (task_) {
      task_->release();              // 3. free EtherCAT resources
    }

    if (sockfd_ >= 0) {
      ::close(sockfd_);
      sockfd_ = -1;
    }

    task_.reset();                   // 4. drop the object
  }

  ~TeleopNode() override {
    stop();
  }

private:
  void publish_joint_states() {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = this->get_clock()->now();

      // Example: 2 joints with dummy values
      msg.name = {"J_arm_l_01", "J_arm_l_02", "J_arm_l_03", "J_arm_l_04", "J_arm_l_05", "J_arm_l_06", "J_arm_l_07",
                  "J_arm_r_01", "J_arm_r_02", "J_arm_r_03", "J_arm_r_04", "J_arm_r_05", "J_arm_r_06", "J_arm_r_07",
                 };

      {
        std::scoped_lock lk(ctx.current_m);
        msg.position = {ctx.q_rot[2], ctx.q_rot[3], ctx.q_rot[4], ctx.q_rot[5], ctx.q_rot[6], ctx.q_rot[7], ctx.q_rot[8],
                        ctx.q_rot[9], ctx.q_rot[10],ctx.q_rot[11],ctx.q_rot[12],ctx.q_rot[13],ctx.q_rot[14],ctx.q_rot[15],
                       };
      }

      joint_state_pub_->publish(msg);
    }

  // ROS2 related declaration
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_posi_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joint_enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};

  // params
  std::string file_name_;
  int64_t cycle_time_{};
  int64_t shift_time_{};
  int log_level_{};
  int priority_{};
  int affinity_{};
  int op_mode_{};
  int master_id_{};

  // EtherCAT + IO
  std::unique_ptr<ecat::task> task_;
  sockaddr_in broadcast_addr_{};
  int sockfd_{-1};
  std::thread waiter_;

  // helpers
  bool is_rot(int id) {
    static constexpr int list[] = {7,8,14,15}; 
    for (int e : list) if (e == id) return false;
    return true;
}
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);          // installs SIGINT handler that triggers shutdown
  auto node = std::make_shared<TeleopNode>();
  rclcpp::spin(node);                // returns on Ctrl+C
  node->stop();                      // make sure EtherCAT is released & thread joined
  rclcpp::shutdown();
  return 0;
}
