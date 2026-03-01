#include "mecca_hardware_interface/mecca_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <cstdio>

namespace mecca_hardware_interface
{

static const char *      HW_VERSION = "4.1.0";
static const rclcpp::Logger LOG = rclcpp::get_logger("MeccaHardware");

// ── on_init ──────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
MeccaHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  info_ = info;

  hw_states_.resize(info_.joints.size());
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (const auto & [name, value] : info_.hardware_parameters) {
    if      (name == "serial_port")        port_name_          = value;
    else if (name == "wheel_radius")       wheel_radius_        = std::stod(value);
    else if (name == "wheel_separation_x") wheel_separation_x_  = std::stod(value);
    else if (name == "wheel_separation_y") wheel_separation_y_  = std::stod(value);
    else if (name == "hw_debug")           hw_debug_ = (value == "true" || value == "1");
  }

  RCLCPP_INFO(LOG, "=== MeccaHardware v%s ===",    HW_VERSION);
  RCLCPP_INFO(LOG, "  serial_port        : %s",     port_name_.c_str());
  RCLCPP_INFO(LOG, "  wheel_radius       : %.4f m", wheel_radius_);
  RCLCPP_INFO(LOG, "  wheel_separation_x : %.4f m", wheel_separation_x_);
  RCLCPP_INFO(LOG, "  wheel_separation_y : %.4f m", wheel_separation_y_);
  RCLCPP_INFO(LOG, "  hw_debug           : %s",     hw_debug_ ? "ON" : "OFF");
  RCLCPP_INFO(LOG, "  joints             : %zu",    info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    RCLCPP_INFO(LOG, "    joint[%zu] = %s", i, info_.joints[i].name.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export interfaces ────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
MeccaHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    si.emplace_back(info_.joints[i].name,
      hardware_interface::HW_IF_POSITION, &hw_states_[i].position);
    si.emplace_back(info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY, &hw_states_[i].velocity);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
MeccaHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ci.emplace_back(info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return ci;
}

// ── on_activate ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
MeccaHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(LOG, "Activating — opening %s ...", port_name_.c_str());

  if (openSerialPort()) {
    RCLCPP_INFO(LOG, "Serial port open (fd=%d). Hardware ready.", serial_port_);
  } else {
    RCLCPP_ERROR(LOG,
      "Could not open %s — check USB cable and udev rule. "
      "read() and write() will be no-ops until port is available.",
      port_name_.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
MeccaHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(LOG, "Deactivating — sending stop, closing port.");
  closeSerialPort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── read ─────────────────────────────────────────────────────────────────────
// Drains the serial receive buffer and parses any complete "I ENC" lines.
// Does NOT send anything — all serial writes are in write() to keep the
// STM32 command buffer from receiving two commands in one 33ms window.

hardware_interface::return_type
MeccaHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_port_ < 0) return hardware_interface::return_type::OK;

  int bytes_available = 0;
  ioctl(serial_port_, FIONREAD, &bytes_available);

  // hw_debug: log FIONREAD once per second so we can tell if STM32 is sending anything
  if (hw_debug_) {
    static int fionread_log_cnt = 0;
    if (++fionread_log_cnt >= 30) {
      fionread_log_cnt = 0;
      RCLCPP_INFO(LOG, "read(): FIONREAD=%d bytes in buffer", bytes_available);
    }
  }

  if (bytes_available > 0) {
    char buf[512];
    int n = ::read(serial_port_, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      serial_buffer_ += buf;

      int safety = 20;
      size_t pos;
      while ((pos = serial_buffer_.find_first_of("\r\n")) != std::string::npos
             && safety-- > 0)
      {
        std::string line = serial_buffer_.substr(0, pos);
        serial_buffer_.erase(0, pos + 1);

        if (!line.empty() && line.find("I ENC") != std::string::npos) {
          parseEncoderData(line);
        } else if (!line.empty() && line.find("WATCHDOG") != std::string::npos) {
          RCLCPP_WARN(LOG, "STM32: %s", line.c_str());
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

// ── write ────────────────────────────────────────────────────────────────────
// Alternates every cycle between:
//   Even cycles — send velocity command "V vx vy vz\r\n"
//   Odd  cycles — send encoder poll    "I ENC\r\n"
//
// This ensures the STM32 never receives both commands back-to-back in the
// same ~33ms window. Its single-slot command buffer would silently drop
// the second command if both arrived within one 10ms main-loop period.
//
// V at 15 Hz is well within the 1-second STM32 watchdog timeout.
// Encoder responses are collected by read() in the following even cycle.

hardware_interface::return_type
MeccaHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_port_ < 0) return hardware_interface::return_type::OK;

  if (write_cycle_ % 2 == 0) {
    // ── Even cycle: velocity command ─────────────────────────────────────────
    //
    // hw_commands_[i] = wheel angular velocity in rad/s (set by mecanum controller).
    // Multiply by wheel_radius to get wheel surface speed in m/s.
    // Apply mecanum forward kinematics to get robot body velocity:
    //   vx    = forward speed    (m/s)
    //   vy    = lateral speed    (m/s, +left)
    //   omega = rotation         (rad/s, +CCW)
    // Scale to mm/s and mrad/s for the STM32 "V" command.

    const double fl = hw_commands_[0] * wheel_radius_;   // FL surface speed m/s
    const double fr = hw_commands_[1] * wheel_radius_;   // FR
    const double rl = hw_commands_[2] * wheel_radius_;   // RL
    const double rr = hw_commands_[3] * wheel_radius_;   // RR

    const double vx    = ( fl + fr + rl + rr) / 4.0;
    const double vy    = (-fl + fr + rl - rr) / 4.0;
    const double k     = (wheel_separation_x_ + wheel_separation_y_) / 2.0;
    const double omega = (-fl + fr - rl + rr) / (4.0 * k);

    const int cmd_x   = static_cast<int>(vx    * 1000.0);  // m/s   → mm/s
    const int cmd_y   = static_cast<int>(vy    * 1000.0);  // m/s   → mm/s
    const int cmd_rot = static_cast<int>(omega * 1000.0);  // rad/s → mrad/s

    char buf[64];
    int len = snprintf(buf, sizeof(buf), "V %d %d %d\r\n", cmd_x, cmd_y, cmd_rot);
    int written = ::write(serial_port_, buf, len);
    if (written < 0) {
      RCLCPP_WARN(LOG, "Serial V write failed: %s", strerror(errno));
    }

    // hw_debug: log ~1 Hz (every 15 V sends = 15 × 66ms ≈ 1s)
    if (hw_debug_ && ++log_counter_ >= 15) {
      log_counter_ = 0;
      RCLCPP_INFO(LOG,
        "write(): V %d %d %d  wheels=[%.2f %.2f %.2f %.2f] rad/s",
        cmd_x, cmd_y, cmd_rot,
        hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3]);
    }

  } else {
    // ── Odd cycle: encoder poll ───────────────────────────────────────────────
    // The STM32 will reply "I ENC m1 m2 m3 m4\r\n".
    // read() will collect that response in the next even cycle.
    const char * req = "I ENC\r\n";
    int written = ::write(serial_port_, req, strlen(req));
    if (written < 0) {
      RCLCPP_WARN(LOG, "Serial I ENC write failed: %s", strerror(errno));
    }
  }

  ++write_cycle_;
  return hardware_interface::return_type::OK;
}

// ── parseEncoderData ─────────────────────────────────────────────────────────
// STM32 encoder order: M1=FL, M2=RL, M3=FR, M4=RR  (vendor layout)
// URDF  joint  order:  [0]=FL, [1]=FR, [2]=RL, [3]=RR
// → swap M2 and M3 when mapping to URDF indices.

void MeccaHardware::parseEncoderData(const std::string & line)
{
  long m1 = 0, m2 = 0, m3 = 0, m4 = 0;
  if (sscanf(line.c_str(), "I ENC %ld %ld %ld %ld", &m1, &m2, &m3, &m4) != 4) {
    return;
  }

  rclcpp::Time now = rclcpp::Clock().now();

  // Map ticks to radians using URDF joint order
  double pos[4];
  pos[0] = m1 * RADS_PER_TICK;   // FL (M1)
  pos[1] = m3 * RADS_PER_TICK;   // FR (M3 — swapped)
  pos[2] = m2 * RADS_PER_TICK;   // RL (M2 — swapped)
  pos[3] = m4 * RADS_PER_TICK;   // RR (M4)

  if (first_read_) {
    for (int i = 0; i < 4; ++i) last_enc_pos_[i] = pos[i];
    last_enc_time_ = now;
    first_read_ = false;
    return;   // need two readings to compute velocity
  }

  double dt = (now - last_enc_time_).seconds();
  if (dt <= 0.0) return;

  for (int i = 0; i < 4; ++i) {
    hw_states_[i].velocity = (pos[i] - last_enc_pos_[i]) / dt;
    hw_states_[i].position  = pos[i];
    last_enc_pos_[i]        = pos[i];
  }

  last_enc_time_ = now;
}

// ── Serial helpers ───────────────────────────────────────────────────────────

bool MeccaHardware::openSerialPort()
{
  serial_port_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_port_ < 0) {
    RCLCPP_ERROR(LOG, "open(%s) failed: %s", port_name_.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_port_, &tty) != 0) {
    RCLCPP_ERROR(LOG, "tcgetattr failed: %s", strerror(errno));
    ::close(serial_port_);
    serial_port_ = -1;
    return false;
  }

  cfmakeraw(&tty);
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= (CLOCAL | CREAD);
  tcsetattr(serial_port_, TCSANOW, &tty);
  tcflush(serial_port_, TCIOFLUSH);

  return true;
}

void MeccaHardware::closeSerialPort()
{
  if (serial_port_ >= 0) {
    const char * stop = "V 0 0 0\r\n";
    ::write(serial_port_, stop, strlen(stop));
    ::fsync(serial_port_);
    ::close(serial_port_);
    serial_port_ = -1;
    RCLCPP_INFO(LOG, "Serial port closed.");
  }
}

}  // namespace mecca_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mecca_hardware_interface::MeccaHardware,
  hardware_interface::SystemInterface)
