#include "svf/openocd_config.h"
#include "server/server.h"

std::vector<std::string> OpenocdConfig::chip_name_;
std::vector<std::string> OpenocdConfig::tap_name_;
uint32_t InterfaceConfig::adapter_rate_run = 6000;
bool OpenocdConfig::cortex_initialized = false;
bool UsbJtagConfigInfo::updateCableInfo(CableInfoType type, const std::string& data) {
  if (data == " " || type == KUnknowInfo) {
    LOG_ERROR("Set cable info error!\n");
    return false;
  }
  if (type == kCableSpeed) {
    auto rate = stoi(data);
    set_adapter_rate(rate);
    if (rate > 15000)
      tdo_signal_edge_ = "raising";
    else
    tdo_signal_edge_ = "falling";
  } else if (type == kCableVid) {
    driver_vid_ = stoi(data, nullptr, 16);
  } else if (type == kCablePid) {
    driver_pid_ = stoi(data, nullptr, 16);
  } else if (type == kCableDevDsc) {
    device_desc_ = data;
  } else if (type == kCableIsLayout) {
    is_layout_config_ = true;
    is_layout_config_ = false; // FIXME
  } else if (type == kCableSerNum) {
    serial_num_ = data;
  } else {
    LOG_ERROR("Set cable info error!\n");
    return false;
  }
  return true;
}

bool JlinkConfigInfo::updateCableInfo(CableInfoType type, std::string& data) {
  char temp[100];
  if (data == " " || type == KUnknowInfo) {
    LOG_ERROR("Set cable info error!\n");
    return false;
  } if (type == kCableNsrtAssert) {
      nsrst_assert_ = stoi(data);
  } else if (type == kCableNsrtDelay) {
    nsrst_delay_ = stoi(data);
  } else {
    LOG_ERROR("Set cable info error!\n");
    return false;
  }
  return true;
}

bool InterfaceConfig::interface_command() {
  if (DetectPart::interface_type() == kUnknowCable) {
    LOG_ERROR("interface command error!\n");
    return false;
  }
  std::string interface_name = (DetectPart::interface_type() <= kUsbJtagSMT2Cable) ? "ftdi" : "jlink";
  int retval;
  char temp[1024];
  sprintf(temp, "interface %s", interface_name.c_str());
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool InterfaceConfig::interface_transport_select(TransportType port_type) {
  if (port_type == kUnknowTransport) {
    LOG_ERROR("transport select command error!\n");
    return false;
  }
  std::string transport_name = (port_type == kJtagTransport) ? "jtag" : "swd";
  int retval;
  char temp[1024];
  sprintf(temp, "transport select %s", transport_name.c_str());
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool InterfaceConfig::adapter_rate_config(uint32_t rate) {
  char temp[1024];
  int retval;
  sprintf(temp, "adapter_khz %d", rate);
  adapter_rate_run = rate;
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool USBJTAGConfig::ftdi_vid_pid(uint32_t vid, uint32_t pid) {
  int retval;
  char temp[1024];
  sprintf(temp, "ftdi_vid_pid 0x%x 0x%x", vid, pid);
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool USBJTAGConfig::ftdi_tdo_sample_edge(const std::string &edge_type) {
  int retval;
  char temp[1024];
  sprintf(temp, "ftdi_tdo_sample_edge %s", edge_type.c_str());
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool USBJTAGConfig::ftdi_device_desc(const std::string& device_name) {
  int retval;
  char temp[1024];
  sprintf(temp, "ftdi_device_desc %s", device_name.c_str());
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool USBJTAGConfig::ftdi_serial(const std::string& serial_number) {
  int retval;
  char temp[1024];
  sprintf(temp, "ftdi_serial %s", serial_number.c_str());
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool USBJTAGConfig::ftdi_layout_config(uint32_t init_data, uint32_t init_direction) {
  //uint32_t init_data = 0x00e8;//0x0088 0x2088 0x8008
  //uint32_t init_direction = 0x60eb;//0x008b 0x3f8b 0x800b
  if (DetectPart::interface_type() == kUsbJtagHs1Cable) {
    init_data = 0x2088;
    init_direction = 0x3f8b;
  }
  const char* signal_name[] = { "nSRST", "GPIO0", "GPIO1", "GPIO2"};
  const uint32_t signal_data[] = {0x2000, 0x0100, 0x0200, 0x2000};
  char temp[1024];
  int retval;
  sprintf(temp, "ftdi_layout_init 0x%x 0x%x", init_data, init_direction);
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  for (int cnt = 0; cnt < 4; cnt++) {
    char temp[1024];
    sprintf(temp, "ftdi_layout_signal %s -data 0x%x", signal_name[cnt], signal_data[cnt]);
    retval = command_run_line(temp);
    if (retval != ERROR_OK)
      return false;
  }

  return true;
}

bool USBJTAGConfig::reset_config() {
  char * rst_type = "srst_only";
  char * srst_type = "srst_push_pull";

  char temp[1024];
  int retval;
  sprintf(temp, "reset_config %s %s", rst_type, srst_type);
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool USBJTAGConfig::execute_interfaceConfig() {
  if (DetectPart::interface_type() == kUnknowCable) {
    return false;
  }
  std::string cable_info;
  CHECK_RET(interface_command());
  CHECK_RET(interface_transport_select());
  CHECK_RET(ftdi_vid_pid(usbjtag_info_->driver_vid_, usbjtag_info_->driver_pid_));
  CHECK_RET(ftdi_tdo_sample_edge(usbjtag_info_->tdo_signal_edge_));//raising | falling
  //CHECK_RET(ftdi_serial(usbjtag_info_->serial_num_));
  CHECK_RET(ftdi_device_desc(usbjtag_info_->device_desc_));
  if (usbjtag_info_->is_layout_config_) {
    CHECK_RET(ftdi_layout_config(usbjtag_info_->layout_init_data_, usbjtag_info_->layout_init_direction_));
  }
  CHECK_RET(reset_config());
  CHECK_RET(adapter_rate_config(usbjtag_info_->adapter_rate_));
  return true;
}



bool JlinkConfig::adapter_nsrst_assert(int width) {
  char temp[1024];
  int retval;
  sprintf(temp, "adapter_nsrst_assert_width %d", width);
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}
bool JlinkConfig::adapter_nsrst_delay(int delay_time) {
  char temp[1024];
  int retval;
  sprintf(temp, "adapter_nsrst_delay %d", delay_time);
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool JlinkConfig::reset_config() {
  char temp[1024];
  int retval;
  sprintf(temp, "reset_config trst_and_srst separate srst_nogate connect_deassert_srst");
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool JlinkConfig::execute_interfaceConfig() {
  std::string cable_info;
  if (DetectPart::interface_type() == kUnknowCable) {
    return false;
  }
  CHECK_RET(interface_command());
  CHECK_RET(interface_transport_select());
  CHECK_RET(adapter_nsrst_assert(jlink_info_->nsrst_assert_));
  CHECK_RET(adapter_nsrst_delay(jlink_info_->nsrst_delay_));
  CHECK_RET(adapter_rate_config(jlink_info_->adapter_rate_));
  CHECK_RET(reset_config());
  return true;
}

bool OpenocdConfig::jtag_newtap_config(OpenocdPart* part) {
  char temp[1024] = { 0 };
  int retval;
  if (part->get_chip() == "unknow"){
    chip_name_.push_back("fmsh");
    tap_name_.push_back("auto");
  } else {
    chip_name_.push_back(part->get_chip());
    tap_name_.push_back(part->get_tap());
  }
  //check irlen
  if (part->ir_length() > 10 || part->ir_length() < 2)
    printf("openocd tap irlen error!\n");

  sprintf(temp, "jtag newtap %s %s -irlen %x -ircapture 0x%x -irmask 0x%x -expected-id 0x%x",
      part->get_chip().c_str(), part->get_tap().c_str(), part->ir_length(), 
      part->get_ircapture(), part->get_irmask(), part->idcode());

  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool OpenocdConfig::adapter_rate_config(uint32_t rate) {
  char temp[1024];
  int retval;
  if (DetectPart::interface_type() == kJlinkCable && (rate > 15000 || rate < 0)) {
    printf("Jlink cable not work support in this rate!\n");
    return false;
  }
  if (DetectPart::interface_type() <= kUsbJtagSMT2Cable && (rate > 25000 || rate < 0)) {
    printf("Digilent usb jtag not support work in this rate!\n");
    return false;
  }
  sprintf(temp, "adapter_khz %d", rate);
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool OpenocdConfig::init_command() {
  char temp[1024];
  int retval;
  sprintf(temp, "init");
  retval = command_run_line(temp);
  
  if (retval != ERROR_OK)
    return false;
  return true;

}

bool OpenocdConfig::excute_command(const std::string& cmd_str) {
  char temp[1024];
  int retval;
  sprintf(temp, cmd_str.c_str());
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;

}

bool OpenocdConfig::halt_command() {
  char temp[1024];
  int retval;
  sprintf(temp, "halt");
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;

}
bool OpenocdConfig::scan_chain_command() {
  char temp[1024];
  int retval;
  sprintf(temp, "scan_chain");
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}
bool OpenocdConfig::reset_command() {
  char temp[1024];
  int retval;
  sprintf(temp, "reset");
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;
  return true;
}

bool OpenocdConfig::executeConfig() {
  if (!Jtag_ChainInfo::instance()->is_init_flag()) {
    printf("please boundscan chip first!\n");
    return false;
  }
  if (Jtag_ChainInfo::instance()->is_new_chains() == false) {
    return true;
  }
  clearTapInfo();
  jtag_tap::clear_jtag_all_taps();
  //get chain_part info
  const std::vector<JtagPart*> chains = Jtag_ChainInfo::instance()->chains();
  size_t part_num = chains.size();
  for (size_t cnt = 0; cnt < part_num; cnt++) {
    OpenocdPart* chain = dynamic_cast<OpenocdPart*> (chains[cnt]);
    jtag_newtap_config(chain);
  }
  return true;
}

ZynqConfig::ZynqConfig() : is_target_create_(false) {
  Jtag_ChainInfo* jtag_chain = Jtag_ChainInfo::instance();
  if (!jtag_chain->is_init_flag()) {
    printf("please boundary scan chip first!\n");
  }
}

ZynqConfig::~ZynqConfig() {

}

bool ZynqConfig::target_create_config(std::string target_type, int core_num, const uint32_t* dbgbase) {
  //check dap exist
  if (tap_name_.size() == 0) {
    printf("tap not declaration before configure\n");
    return false;
  }
  auto itr = find(tap_name_.begin(), tap_name_.end(), "dap");
  if (itr == tap_name_.end() || core_num < 1) {
    printf("chip not detected cpu info\n");
    return false;
  }
  auto cnt = itr - tap_name_.begin();
  std::string chip_name = chip_name_[cnt];
  std::string core_name;

  char temp[1024];
  char str_num[10];
  int retval;
  for (int i = 0; i < core_num; ++i) {
	sprintf(str_num, "%d", i);
    core_name = chip_name + ".cpu" + std::string(str_num);
    core_construct_.push_back(core_name);
    sprintf(temp, "target create %s.cpu%d %s -chain-position %s.dap -coreid %d -dbgbase 0x%x", chip_name.c_str(), i, target_type.c_str(), chip_name.c_str(), i, dbgbase[i]);
    retval = command_run_line(temp);
    if (retval != ERROR_OK)
      return false;
  }
  is_target_create_ = true;
  return true;
}

bool ZynqConfig::cpu_configure_command_config(const std::string& event_name, const std::string& event_body) {
  if (!is_target_create_) {
    printf("target not create before configure\n");
    return false;
  }
  char temp[1024];
  int retval;
  for (size_t i = 0; i < core_construct_.size(); ++i) {
    sprintf(temp, "%s configure -event %s %s", core_construct_[i].c_str(), event_name.c_str(), event_body.c_str());
    retval = command_run_line(temp);
    if (retval != ERROR_OK)
      return false;
  }

  return true;
}

bool ZynqConfig::target_smp_config() {
  if (core_construct_.size() == 1)
    return true;
  if (!is_target_create_ || core_construct_.size() == 0) {
    printf("target not create before configure\n");
    return false;
  }
  char temp[1024];
  std::string temp_str;
  int retval;
  for (size_t i = 0; i < core_construct_.size(); ++i) {
	temp_str += core_construct_[i] + " ";
  }
  sprintf(temp, "target smp %s", temp_str.c_str());//FIXME
  retval = command_run_line(temp);
  if (retval != ERROR_OK)
    return false;

  try {
  }catch (const char* msg){
	  throw("error msg when excute!\n");
  }
  return true;
}

bool ZynqConfig::executeTargetDapConfig() {
  //zynq7010--hardcode
  const uint32_t dbgbase[] = {
    //0x80090000,
    //0x80092000
    0x80090000,
    0x80092000,
    0x80094000,
    0x80096000
  };
  int core_num = OpenocdPart::core_num_;
  CHECK_RET(target_create_config(std::string("cortex_a"), core_num, dbgbase));
  CHECK_RET(target_smp_config());
  CHECK_RET(cpu_configure_command_config());
  CHECK_RET(init_command());
  cortex_initialized = true;
#ifdef _DEBUG
  LOG_INFO("zynq cortex_a arm core configure success");
#endif
  return true;
}


