#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "ecat/task.hpp"
#include "../power.hpp"
#include "../io.hpp"
#include "ecat/s2s_func.hpp"
#include "../CtrlContext.hpp"
#include "Joint.hpp"
#include "WaveHandDemo.hpp"

#include <iostream>
#include <qiuniu/init.h>
#include <spdlog/spdlog.h>
#include <vector>
#include <boost/program_options.hpp>
#include <signal.h>
#include <sys/mman.h>
#include <thread>

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

void signal_handler(int sig){
  shutdownAllJoints();
  signal(SIGINT, SIG_DFL);
  tt->break_();
}

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("upper_body_demo_node");
  auto publisher = node->create_publisher<std_msgs::msg::String>("status", 10);

  // Spin ROS in background
  std::thread ros_thread([&]() { rclcpp::spin(node); });

  // Notify start
  std_msgs::msg::String msg;
  msg.data = "Upper Body Demo Node started";
  publisher->publish(msg);

  // EtherCAT setup (original logic)
  std::atexit(shutdownAllJoints);
  joints.reserve(16);
  signal(SIGINT, signal_handler);
  qiuniu_init();  

  if (mlockall(MCL_FUTURE | MCL_CURRENT) == -1)
  {
    perror("failed to lock memory\n");
    return 1;
  }
  
  // spdlog::set_level(spdlog::level::trace);

  std::string file_name;
  std::int64_t cycle_time;
  std::int64_t shiftTime;
  boost::program_options::options_description opts("all option");
  boost::program_options::variables_map vm;
  int log_level = 0;
  int priority = 90;
  int affinity = 0;
  int op_mode = 0;
  int master_id = 0;

  opts.add_options()
      ("help,h", "This is EtherCAT Demo program.")
      ("fileName,f", boost::program_options::value<std::string>(&file_name)->default_value(""), "Set eni xml, only for eni mode.")
      ("cycleTime,c", boost::program_options::value<std::int64_t>(&cycle_time)->default_value(1000000), "Set cycle time(ns), only for esi mode.")
      ("shiftTime,s", boost::program_options::value<std::int64_t>(&shiftTime)->default_value(300000), "Set cycle shift time(ns), only for esi mode.")
      ("log,l", boost::program_options::value<int>(&log_level)->default_value(0), "Set log level, (0-6).")
      ("priority,p", boost::program_options::value<int>(&priority)->default_value(90), "Set priority of the realtime task (1 - 99)")
      ("affinity,a", boost::program_options::value<int>(&affinity)->default_value(1), "Set CPU affinity of the realtime task")
      ("op_mode,o", boost::program_options::value<int>(&op_mode)->default_value(9), "Set operation mode to csv")
      ("master_id,m", boost::program_options::value<int>(&master_id)->default_value(0), "Set master");
  
  try
  {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    vm.notify();
  }
  catch(std::exception &e)
  {
    std::cout << e.what() << "\n";
    std::cout << opts << "\n";
    return 1;
  }

  if (vm.count("help"))
  {
    std::cout << opts << "\n";
    usage(argv[0]);
    return 0;
  }

  spdlog::set_level(static_cast<spdlog::level::level_enum>(log_level));

  ecat::task task(master_id);
  task.priority(priority);
  tt = &task;

  cpu_set_t cpus;
  CPU_ZERO(&cpus);

  CPU_SET(affinity, &cpus);
  task.cpu_affinity(&cpus, sizeof(cpus));
  task.record(false);

  //esi模式下设置默认的循环周期和同步模式
  std::cout << "Use esi xml\n";
  task.cycle_time(cycle_time, shiftTime);
  task.dc_mode(ecat::dc_mode::master_follow_slave);

  //设置config的回调函数，将在task.start的时候被到用
  task.set_config_callback([&] {
    std::uint16_t slave_count = task.slave_count();
    std::uint16_t slave_pos = 0;
    for (slave_pos = 0; slave_pos < slave_count; slave_pos++)
    {
      //获取slave对应的运行协议，如果还未设置则默认为0
      auto profile_no = task.profile_no(slave_pos);
      //CiA402协议模式
      if (profile_no == 402)
      {
        int n_axis_in_slave = 1, slot_pos;
        int slots_count = task.slots_count(slave_pos);
        int slots_index_increment = task.slot_index_increment(slave_pos);
        if (slots_count > 0)
        {
          assert(slots_index_increment != -1);
          n_axis_in_slave = slots_count;
        }
        assert(!(slots_count < 0));
        for (slot_pos = 0; slot_pos < n_axis_in_slave; ++slot_pos)
        {
          const int index_offset = slot_pos * slots_index_increment;
          auto axis = std::make_unique<axis_data>();
          axis->slave_pos = slave_pos;
          axis->axis_id = axis_count++;

          //获取PDO对应的domain中的地址偏移量，可根据实际需要添加
          task.try_register_pdo_entry(axis->control_word, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6040 + index_offset), 0}); // control word;
          task.try_register_pdo_entry(axis->target_position, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x607a + index_offset), 0}); // target position
          task.try_register_pdo_entry(axis->target_velocity, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x60ff+ index_offset), 0}); // target velocity    
          task.try_register_pdo_entry(axis->target_torque, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6071+ index_offset), 0}); // target torque                                
          task.try_register_pdo_entry(axis->mode_of_operation, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6060 + index_offset), 0}); // mode of operation
          task.try_register_pdo_entry(axis->status_word, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6041 + index_offset), 0}); // status word  
          task.try_register_pdo_entry(axis->position_actual_value, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6064 + index_offset), 0}); // position actual value   
          task.try_register_pdo_entry(axis->velocity_actual_value, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x606c + index_offset), 0}); // veocity actual value  
          task.try_register_pdo_entry(axis->torque_actual_value, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6077 + index_offset), 0}); // torque actual value                                                                                                                                                  
          task.try_register_pdo_entry(axis->mode_of_operation_display, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6061 + index_offset), 0}); // mode of operation display   
          task.try_register_pdo_entry(axis->error_code, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x603f + index_offset), 0}); // error_code 
          // Setup Joints
          axis->mode = op_mode;
          axis_data* ptr = axis.get();
          joints.push_back(std::make_unique<Joint>(*ptr, ctx));
          axes.push_back(std::move(axis));
        }
      }
    }
    ecat::S2SConfig::use().count();
  });

  // Receive callback
  task.set_receive_callback([&] {
    for (auto &j : joints)
    {
      (*j)();
    }
    publisher->publish(msg);
    // Check if all joints finished and advance step
    ctx.all_joints_done = true;
    for (int i = 0; i < num_joints; ++i) {
        if (!ctx.joint_done[i]) {
            ctx.all_joints_done = false;
            break;
        }
    }

    if (ctx.all_joints_done) {
        if (ctx.step_index < num_steps - 1) {
            ctx.step_index++;
            printf("\n\n#############################\n[STEP] Advancing to step %d....\n#############################\n\n", ctx.step_index);
        } else {
            printf("[STEP] Final step reached. Holding position...\n");
            ctx.end = true;
            // tt->break_();
            // std::exit(0);
        }

        for (int i = 0; i < num_joints; ++i) {
            ctx.joint_done[i] = false;
            ctx.plan_flag[i] = true;
        }
    }
    
    // Check if all joints are able to move
    ctx.all_move_allowed = true;
    for (int i = 0; i < num_joints; ++i) {
      if (!ctx.move_allowed[i]) {
          // printf("Stopped due to %d\n", i);
          ctx.all_move_allowed = false;
          break;
        }
      }
  });

  task.record(false);
  task.start();
  task.wait();
  task.release();

  // Shutdown ROS
  rclcpp::shutdown();
  ros_thread.join();
  return 0;
}