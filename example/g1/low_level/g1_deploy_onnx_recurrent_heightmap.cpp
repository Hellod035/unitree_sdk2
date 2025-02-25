#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <pthread.h>
#include <sched.h>
#include "gamepad.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include "unitree/idl/ros2/String_.hpp"

// ONNX
#include <onnxruntime_cxx_api.h>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";
static const std::string MAP_TOPIC = "height_map/height_map_data";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;
using namespace std_msgs::msg::dds_;

std::array<float, 187> parseArrayFromString(const std::string& str) {
    std::vector<float> result;
    std::array<float, 187> arr;
    std::istringstream iss(str);
    char ch;
    float value;
    iss >> ch;
    while (iss >> value) {
        result.push_back(value);
        iss >> ch;
    }
    std::copy(result.begin(), result.end(), arr.begin());
    return arr;
}

template <typename T>
class DataBuffer
{
public:
  void SetData(const T &newData)
  {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData()
  {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }

  void Clear()
  {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }

private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

std::array<float, 3> GetGravityOrientation(const std::array<float, 4> &quaternion)
{
  float qw = quaternion[0];
  float qx = quaternion[1];
  float qy = quaternion[2];
  float qz = quaternion[3];

  std::array<float, 3> gravity_orientation = {0.0f, 0.0f, 0.0f};

  gravity_orientation[0] = 2.0f * (-qz * qx + qw * qy);
  gravity_orientation[1] = -2.0f * (qz * qy + qw * qx);
  gravity_orientation[2] = 1.0f - 2.0f * (qw * qw + qz * qz);

  return gravity_orientation;
}

const int G1_NUM_MOTOR = 29;

const std::array<float, 3> command_max = {0.7f, 0.4f, 1.57f};
const std::array<float, 3> command_min = {-0.4f, -0.4f, -1.57f};

struct MotorCommand
{
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};

enum class Mode
{
  PR = 0, // Series Control for Ptich/Roll Joints
  AB = 1  // Parallel Control for A/B Joints
};

const std::array<int, 29> joint2motor_idx = {
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22,
    4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19,
    26, 20, 27, 21, 28};

const std::array<int, 29> kps = {
    200, 200, 200, 150, 150, 200, 150, 150, 200,
    200, 200, 100, 100, 20, 20, 100, 100, 20,
    20, 50, 50, 50, 50, 40, 40, 40, 40, 40, 40};

const std::array<int, 29> kds = {
    5, 5, 5, 5, 5, 5, 5, 5, 5,
    5, 5, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

const std::array<double, 29> default_angles = {
    -0.2, -0.2, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.42, 0.42, 0.35, 0.35,
    -0.23, -0.23, 0.18, -0.18, 0.0, 0.0,
    0.0, 0.0, 0.87, 0.87, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0};

enum class ProgramState
{
  INIT,
  WAIT_FOR_CONTROL,
  CONTROL
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len)
{
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++)
  {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++)
    {
      if (CRC32 & 0x80000000)
      {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      }
      else
        CRC32 <<= 1;
      if (data & xbit)
        CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
};

class G1Example
{
private:

  double time_;
  double publish_dt_;
  double control_dt_;
  double duration_;   // [3 s]
  int counter_;
  Mode mode_pr_;
  uint8_t mode_machine_;

  Gamepad gamepad_;
  REMOTE_DATA_RX rx_;

  DataBuffer<LowState_> low_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<std::array<float, 187>> map_data_buffer_;

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelSubscriberPtr<IMUState_> imutorso_subscriber_;
  ChannelSubscriberPtr<String_> map_subscriber_;
  ThreadPtr command_writer_ptr_, control_thread_ptr_;

  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

  ProgramState program_state_;
  std::array<float, 29> last_action;

  Ort::Env env;
  Ort::Session* policy_session;
  Ort::AllocatorWithDefaultOptions allocator;
  std::vector<const char*> input_node_names;
  std::vector<const char*> output_node_names;
  std::vector<int64_t> input_node_dims;

  std::array<float, 256> h_state = {0.0f};
  std::array<float, 256> c_state = {0.0f};

  bool use_flat_heightmap_ = true;
  bool heightmap_init_done = false;

public:
  bool stop_flag = false;
  G1Example(std::string networkInterface)
      : time_(0.0),
        publish_dt_(0.002),
        control_dt_(0.02),
        duration_(3.0),
        counter_(0),
        mode_pr_(Mode::PR),
        mode_machine_(0),
        program_state_(ProgramState::INIT),
        last_action{0.0f},
        env(ORT_LOGGING_LEVEL_WARNING, "G1Example")
  {
    ChannelFactory::Instance()->Init(0, networkInterface);

    // try to shutdown motion control-related service
    msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(5.0f);
    msc_->Init();
    std::string form, name;
    while (msc_->CheckMode(form, name), !name.empty())
    {
      if (msc_->ReleaseMode())
        std::cout << "Failed to switch to Release Mode\n";
      sleep(5);
    }

    // create publisher
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();
    // create subscriber
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);

    // create map subscriber
    map_subscriber_.reset(new ChannelSubscriber<String_>(MAP_TOPIC));
    map_subscriber_->InitChannel(std::bind(&G1Example::MapDataHandler, this, std::placeholders::_1), 1);
  
    // create threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, publish_dt_*1e6, &G1Example::LowCommandWriter, this);
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, control_dt_*1e6, &G1Example::Control, this);

    try {
        policy_session = new Ort::Session(env, "policy.onnx", Ort::SessionOptions{nullptr});
        Ort::SessionOptions policy_session_options;
        policy_session_options.SetIntraOpNumThreads(1);
        policy_session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        input_node_names.push_back("obs");
        input_node_names.push_back("h_in");
        input_node_names.push_back("c_in");
        output_node_names.push_back("actions");
        output_node_names.push_back("h_out");
        output_node_names.push_back("c_out");
        std::cout << "Successfully loaded policy model" << std::endl;
    } catch (const Ort::Exception& e) {
        std::cerr << "Error loading policy model: " << e.what() << std::endl;
    }

    SetThreadPriority();
  }

  ~G1Example() {
    delete policy_session;
  }

  void SetThreadPriority() {
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  }

  void LowStateHandler(const void *message)
  {
    // std::cout << "LowStateHandler" << std::endl;
    LowState_ low_state = *(const LowState_ *)message;
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1))
    {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;
    }

    low_state_buffer_.SetData(low_state);

    // update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    gamepad_.update(rx_.RF_RX);

    stop_flag = gamepad_.select.pressed;
    if (gamepad_.B.on_release){
        use_flat_heightmap_ = false;
        std::cout << "Use GT HeightMap !" << std::endl;
    }
    if (gamepad_.Y.on_release){
        use_flat_heightmap_ = true;
        std::cout << "Use Flat HeightMap !" << std::endl;
    }
    // update mode machine
    if (mode_machine_ != low_state.mode_machine())
    {
      if (mode_machine_ == 0)
        std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
      mode_machine_ = low_state.mode_machine();
    }
  }

  void LowCommandWriter()
  {
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (mc)
    {
      for (size_t i = 0; i < G1_NUM_MOTOR; i++)
      {
        dds_low_command.motor_cmd().at(i).mode() = 1; // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
        dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
        dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
        dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
        dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
      }

      dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
      lowcmd_publisher_->Write(dds_low_command);
    }
  }

  void MapDataHandler(const void *message)
  {

    String_ map_str = *(const String_ *)message;
    std::array<float, 187> map_data = parseArrayFromString(map_str.data());
    map_data_buffer_.SetData(map_data);
    heightmap_init_done = true;

  }

  void Stop()
  {
    stop_flag = true;

    if (control_thread_ptr_)
    {
      control_thread_ptr_->Wait();
      control_thread_ptr_.reset();
    }
    CreateDampingCommand();
    LowCommandWriter();
    std::cout << "Stop" << std::endl;
  }

  void CreateDampingCommand()
  {
    MotorCommand motor_command_tmp;
    const std::shared_ptr<const LowState_> ls = low_state_buffer_.GetData();

    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.q_target.at(i) = 0.0;
      motor_command_tmp.dq_target.at(i) = 0.0;
      motor_command_tmp.kp.at(i) = 0;
      motor_command_tmp.kd.at(i) = 8;
    }
    motor_command_buffer_.SetData(motor_command_tmp);
  }

  void Control()
  {
    if (stop_flag)
    {
      return;
    }
    MotorCommand motor_command_tmp;
    const std::shared_ptr<const LowState_> ls = low_state_buffer_.GetData();

    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
      motor_command_tmp.tau_ff.at(joint2motor_idx[i]) = 0.0;
      motor_command_tmp.q_target.at(joint2motor_idx[i]) = default_angles[i];
      motor_command_tmp.dq_target.at(joint2motor_idx[i]) = 0.0;
      motor_command_tmp.kp.at(joint2motor_idx[i]) = kps[i];
      motor_command_tmp.kd.at(joint2motor_idx[i]) = kds[i];
    }

    if (ls)
    {
      switch (program_state_)
      {
      case ProgramState::INIT:
        time_ += control_dt_;
        if (time_ < duration_)
        {
          for (int i = 0; i < G1_NUM_MOTOR; i++)
          {
            double ratio = std::clamp(time_ / duration_, 0.0, 1.0);
            double current_pos = ls->motor_state()[joint2motor_idx[i]].q();
            motor_command_tmp.q_target.at(joint2motor_idx[i]) = current_pos * (1.0 - ratio) + default_angles[i] * ratio;
          }
        }
        else
        {
          program_state_ = ProgramState::WAIT_FOR_CONTROL;
          std::cout << "Init Done" << std::endl;
        }
        break;

      case ProgramState::WAIT_FOR_CONTROL:
        if (gamepad_.A.pressed)
        {
          program_state_ = ProgramState::CONTROL;
          std::cout << "Start Control" << std::endl;
        }
        break;

      case ProgramState::CONTROL:
        // auto start = std::chrono::high_resolution_clock::now();
        const std::array<::unitree_hg::msg::dds_::MotorState_, 35> motor_stats = ls->motor_state();
        std::array<float, G1_NUM_MOTOR> q;
        std::array<float, G1_NUM_MOTOR> dq;
        for (int i = 0; i < G1_NUM_MOTOR; i++)
        {
          q.at(i) = motor_stats[joint2motor_idx[i]].q() - default_angles[i];
          dq.at(i) = motor_stats[joint2motor_idx[i]].dq();
        }

        const std::array<float, 4> quat = ls->imu_state().quaternion();
        const std::array<float, 3> ang_vel = ls->imu_state().gyroscope();
        const std::array<float, 3> projected_gravity = GetGravityOrientation(quat);

        std::array<float, 3> command = {gamepad_.ly, -gamepad_.lx, -gamepad_.rx};
        for (int i = 0; i < 3; i++) {
          command[i] = std::clamp(command[i], command_min[i], command_max[i]);
        }

        std::array<float, 283> obs;

        std::array<float, 187> heightmap_data_flat;
        std::fill(heightmap_data_flat.begin(), heightmap_data_flat.end(), 0.322f);

        std::copy(ang_vel.begin(), ang_vel.end(), obs.begin());
        std::copy(projected_gravity.begin(), projected_gravity.end(), obs.begin() + 3);
        std::copy(command.begin(), command.end(), obs.begin() + 6);
        std::copy(q.begin(), q.end(), obs.begin() + 9);
        std::copy(dq.begin(), dq.end(), obs.begin() + 9 + 29);
        std::copy(last_action.begin(), last_action.end(), obs.begin() + 9 + 29 * 2);

        if (use_flat_heightmap_ || !heightmap_init_done) {
          std::copy(heightmap_data_flat.begin(), heightmap_data_flat.end(), obs.begin() + 9 + 29 * 3);
        } else {
          if (const auto& heightmap_data_ptr = map_data_buffer_.GetData()) {
            const auto& heightmap_data = *heightmap_data_ptr;
            std::copy(heightmap_data.begin(), heightmap_data.end(), obs.begin() + 9 + 29 * 3);
          }
        }

        // std::cout << "last obs:" << obs[obs.size()-1] << std::endl;
        std::vector<float> input_tensor_values(obs.begin(), obs.end());

        std::vector<int64_t> obs_dims = {1, static_cast<int64_t>(obs.size())};
        Ort::Value obs_tensor = Ort::Value::CreateTensor<float>(allocator.GetInfo(), input_tensor_values.data(), input_tensor_values.size(), obs_dims.data(), obs_dims.size());

        std::vector<int64_t> h_dims = {1, 1, 256};
        Ort::Value h_tensor = Ort::Value::CreateTensor<float>(allocator.GetInfo(), h_state.data(), h_state.size(), h_dims.data(), h_dims.size());

        std::vector<int64_t> c_dims = {1, 1, 256};
        Ort::Value c_tensor = Ort::Value::CreateTensor<float>(allocator.GetInfo(), c_state.data(), c_state.size(), c_dims.data(), c_dims.size());

        std::vector<Ort::Value> input_tensors;
        input_tensors.emplace_back(std::move(obs_tensor));
        input_tensors.emplace_back(std::move(h_tensor));
        input_tensors.emplace_back(std::move(c_tensor));

        auto output_tensors = policy_session->Run(Ort::RunOptions{nullptr},
                                                 input_node_names.data(),
                                                 input_tensors.data(),
                                                 input_tensors.size(),
                                                 output_node_names.data(),
                                                 output_node_names.size());

        float* floatarr = output_tensors[0].GetTensorMutableData<float>();

        float* h_out = output_tensors[1].GetTensorMutableData<float>();
        float* c_out = output_tensors[2].GetTensorMutableData<float>();
        std::copy(h_out, h_out + 256, h_state.begin());
        std::copy(c_out, c_out + 256, c_state.begin());

        for (int i = 0; i < G1_NUM_MOTOR; i++)
        {
          const float action_value = floatarr[i] * 0.25f;
          last_action[i] = floatarr[i];
          motor_command_tmp.q_target.at(joint2motor_idx[i]) = default_angles[i] + action_value;
        }
        // auto end = std::chrono::high_resolution_clock::now();
        // std::cout << "end: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "μs" << std::endl;
      }
      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }
};

int main(int argc, char const *argv[])
{
  if (argc < 2)
  {
    std::cout << "Usage: g1_deploy_onnx_recurrent network_interface" << std::endl;
    exit(0);
  }
  std::string networkInterface = argv[1];
  G1Example custom(networkInterface);
  while (!custom.stop_flag)
  {
    sleep(0.02);
  }
  custom.Stop();
  return 0;
}
