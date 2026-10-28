#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <fstream>
#include <vector>
#include <cstdint>

// WAV 文件头结构体
struct WavHeader
{
  char riff[4] = {'R', 'I', 'F', 'F'};
  uint32_t chunk_size;
  char wave[4] = {'W', 'A', 'V', 'E'};
  char fmt[4] = {'f', 'm', 't', ' '};
  uint32_t subchunk1_size = 16;
  uint16_t audio_format = 1;  // PCM
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char data[4] = {'d', 'a', 't', 'a'};
  uint32_t data_size;
};

class MicRecorder : public rclcpp::Node
{
 public:
  MicRecorder() : Node("mic_recorder"), sample_rate_(16000), num_channels_(8)
  {
    sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/sys/speech/mic_source", 10,
        std::bind(&MicRecorder::onMicData, this, std::placeholders::_1));

    wav_file_.open("mic_output.wav", std::ios::binary);
    if (!wav_file_)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output file");
      return;
    }

    // 先写一个占位的 WAV 头，之后会回填大小
    writeEmptyHeader();
    RCLCPP_INFO(this->get_logger(), "Recording started...");
  }

  ~MicRecorder()
  {
    finalizeWav();
    if (wav_file_.is_open()) wav_file_.close();
    RCLCPP_INFO(this->get_logger(),
                "Recording stopped. Saved to mic_output.wav");
  }

 private:
  void onMicData(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    // 将数据直接写入文件
    wav_file_.write(reinterpret_cast<const char *>(msg->data.data()),
                    msg->data.size() * sizeof(int16_t));
    data_bytes_ += msg->data.size() * sizeof(int16_t);
  }

  void writeEmptyHeader()
  {
    WavHeader header;
    header.chunk_size = 0;
    header.num_channels = num_channels_;
    header.sample_rate = sample_rate_;
    header.bits_per_sample = 16;
    header.byte_rate =
        sample_rate_ * num_channels_ * header.bits_per_sample / 8;
    header.block_align = num_channels_ * header.bits_per_sample / 8;
    header.data_size = 0;

    wav_file_.write(reinterpret_cast<const char *>(&header), sizeof(WavHeader));
  }

  void finalizeWav()
  {
    if (!wav_file_) return;

    wav_file_.seekp(0, std::ios::beg);

    WavHeader header;
    header.num_channels = num_channels_;
    header.sample_rate = sample_rate_;
    header.bits_per_sample = 16;
    header.byte_rate =
        sample_rate_ * num_channels_ * header.bits_per_sample / 8;
    header.block_align = num_channels_ * header.bits_per_sample / 8;
    header.data_size = data_bytes_;
    header.chunk_size = 36 + data_bytes_;

    wav_file_.write(reinterpret_cast<const char *>(&header), sizeof(WavHeader));
  }

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_;
  std::ofstream wav_file_;
  size_t data_bytes_ = 0;
  uint32_t sample_rate_;
  uint16_t num_channels_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MicRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
