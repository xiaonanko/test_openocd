#include <fstream>
#include "svf/bin_program.h"
#include "svf/openocd_config.h"
#include "server/openocd_server.h"
#include "helper/time_support.h"
#include "jtag/chain_info.h"
#define DCC_BUFF_SIZE 8192
const char* BootLoad_CMD[] = {
  "?", "base", "cmp", "cp", "env", "help", "h", "loop", "md", "mm", "mw",
  "nm", "printenv", "p", "print", "reset", "setenv", "sf", "version", "v"
};

const char* ZynQspiType[] = {
  "nand_flash", "nor_flash", "qspi_x1_single", "qspi_x2_single", "qspi_x4_single",
  "qspi_x1_dual_stacked", "qspi_x2_dual_stacked", "qspi_x4_dual_stacked",
  "qspi_x8_dual_parallel", "unknow_flash_type" 
};

const uint32_t ZynqInternalRegisterMap[] = { /*BOOT_MODE_REG*/0xF800025C,
    /*ARM_PLL_CFG*/0xF8000110, /*ARM_PLL_CTRL*/0xF8000100, /*ARM_CLK_CTRL*/0xF8000120, 
    /*IO_PLL_CFG*/0xF8000118, /*IO_PLL_CTRL*/0xF8000108, /*kOCM_ENABLE_REG*/0xF8000008,
    /*kOCM_CFG_REG*/0xF8000910, /*kOCM_CTRL_REG*/0xF8000004
};

bool BootloaderConsole::jtagTerminalStart() {
  int ret;
  std::string str_in;
  struct target *target = get_current_target(command_context::cmd_ctx());
  if (!initDccConsole()) {
    return false;
  }
  if (target->state == TARGET_RUNNING) {
    command_print("\nInfo: DCC jtag console start");
  } else {
    command_print("\nInfo: Bootloader not running in targets");
    return false;
  }
  std::cout << "Zynq>";
  while (std::getline(std::cin, str_in)) {
    //when input dcc -stop break function
    if (std::string::npos != str_in.find("-stop")) {
      command_print("DCC jtag console exit!");
      break;
    }
    if (std::string::npos != str_in.find("exit")) {
      openocd_exitstatus_ = true;
      break;
    }
    if (checkBootloaderCommand(str_in.c_str())) {
      ret = dccStreamIn(str_in);
      if (!ret) {
        command_print("dcc console write failed");
        return false;
      }
    } else if (str_in.empty()) {
      std::cout << "Zynq>";
    } else {
      ret = command_run_line(str_in.c_str());
      std::cout << "Zynq>";
    }  
  }

  return true;
} 

bool BootloaderConsole::jtagTerminalStop() {

  return true;
}
//Dcc console operation
bool BootloaderConsole::initDccConsole() {
  bool ratval;
  if (!OpenocdConfig::cortex_initialized) {
    //check part is arm part
    auto current_chain = Jtag_ChainInfo::instance();
    if (!current_chain->is_zynq_chip()) {
      command_print("current chain part not support dcc console!");
      return false;
    }
    ZynqConfig target_configure;
    ratval = target_configure.executeTargetDapConfig();
    if (!ratval) {
      return false;
    }
  }

  LOG_DEBUG("Dcc console initialized");
  return true;
}

//DCC_STDOUT DCC_SDKOUT DCC_NONE
bool BootloaderConsole::dccStreamFlush(FlushMode mode) {
  int ratval;
  char *data = new char[DCC_BUFF_SIZE];
  int len = 0;
  struct target *target = get_current_target(command_context::cmd_ctx());
  ratval = target_dcc_console_read(target, data, &len, mode);
  if (ratval != ERROR_OK)
    return false;
  if (mode == DCC_SDKOUT) {
    std::cout << std::string(data, len) << std::endl;
  }
  delete data;
  return true;
}

bool BootloaderConsole::dccStreamIn(std::string input_str) {
  int ratval;
  //analysis input string 
  int str_size = input_str.size();
  input_str = input_str + "\r";
  struct target *target = get_current_target(command_context::cmd_ctx());

  ratval = target_dcc_console_write(target, input_str.c_str(), str_size+1);
  if (ratval != ERROR_OK)
    return false;
  dccStreamFlush(stream_mode_);
  return true;
}

bool BootloaderConsole::checkBootloaderCommand(const char* cmd) {
  char str[100];
  int pos = 0;
  strcpy(str, cmd);
  for (; pos < strlen(cmd); pos++) {
    if (str[pos] == ' ')
      break;
  }
  str[pos] = '\0';
  for (int i = 0; i < 20; i++) {
    if (strcasecmp(str, BootLoad_CMD[i]) == 0) {
      return true;
    }
  }
  return false;
}

bool ProgramBIN::executeprogram() {
  int ret;
  std::string boot_str("qspi_x8_dual_parallel");
  std::string path_bootloader("zynq_qspi_x8_dual_parallel.bin");
  CHECK_RET(executeElfFile());
  //CHECK_RET(detectBootloaderPath(boot_str, path_bootloader));
  CHECK_RET(executeBootLoader(path_bootloader));
  CHECK_RET(executeBinFile());
  return true;
}

bool ProgramBIN::setAdapterSpeed(uint32_t speed) {
  int retval = jtag_adapter_manager::jtag_config_khz(speed);
  if (ERROR_OK != retval)
    return false;
  int cur_speed = jtag_adapter_manager::jtag_get_speed_khz();
  retval = jtag_adapter_manager::jtag_get_speed_readable(&cur_speed);
  if (ERROR_OK != retval) {
    return false;
  }
  if (cur_speed == 0) {
    command_print("adapter speed seting failed");
    return false;
  }
  return true;
}

//Targets operation
bool ProgramBIN::targetResume(unsigned int addr) {
  int retval = ERROR_OK;
  char temp[100] = {0};
  struct target *target = get_current_target(command_context::cmd_ctx());
  if (target->state != TARGET_HALTED) {
    CHECK_RET(targetHalt(10));
  } 
  do {
    if (addr == -1) {
      retval = command_run_line("resume");
      break;
    }
    sprintf(temp, "resume 0x%x", addr);
    retval = command_run_line(temp);
  } while (0);
  if (retval != ERROR_OK)
    command_print("target resume failed in 0x%x, please reset board\n", addr);
  return true;
}

bool ProgramBIN::targetHalt(int ms) {
  int retval1 = ERROR_OK, retval2 = ERROR_OK;
  struct target *target = get_current_target(command_context::cmd_ctx());
  if (target->state != TARGET_HALTED) {
    retval1 = target_halt(target);
    retval2 = target_wait_state(target, TARGET_HALTED, ms);
    if (ERROR_OK != retval1 || retval2 != ERROR_OK){
      command_print("target halt failed, please reset board\n");
      return false;
    }
  }
  return true;
}

bool ProgramBIN::targetSelectCore() {
  return true;
}

bool ProgramBIN::targetPrint() {
  return true;
}

bool ProgramBIN::targetPolling() {
  return true;
}

bool ProgramBIN::targetReadRegister(TargetRegister reg, uint32_t* buf) {
  uint8_t* data = new uint8_t[4];
  struct target *target = get_current_target(command_context::cmd_ctx());
  //halt target
  if (target->state != TARGET_HALTED) {
    CHECK_RET(targetHalt(10));
  }
  int retval = target_read_memory(target, ZynqInternalRegisterMap[reg], 4, 1, data);
  uint32 status = target_buffer_get_u32(target, data);
  *buf = status;
  if (retval != ERROR_OK) {
    command_print("target read register failed at 0x%x\n", ZynqInternalRegisterMap[reg]);
    return false;
  }
  delete data;
  return true;
}

bool ProgramBIN::targetWriteRegister(TargetRegister reg, uint32_t value) {
  uint8_t* data = new uint8_t[4];
  struct target *target = get_current_target(command_context::cmd_ctx());
  //halt target
  if (target->state != TARGET_HALTED) {
    CHECK_RET(targetHalt(10));
  }
  target_buffer_set_u32(target, data, value);
  int retval = target_write_memory(target, ZynqInternalRegisterMap[reg], 4, 1, data);  
  if (retval != ERROR_OK) {
    command_print("target write register failed at 0x%x\n", ZynqInternalRegisterMap[reg]);
    return false;
  } 
  delete data;
  return true;
}

bool ProgramBIN::targetExcuteImage(struct target *target, uint32_t base_address, uint32_t len, uint8_t* buffer) {
  int retval = ERROR_OK;
  //struct duration bench;
  //duration_start(&bench);
  retval = target_write_buffer(target, base_address, len, buffer);
  if (retval != ERROR_OK)
    return false;
  //duration_measure(&bench);
  //command_print("Program %x size Block image spend %fs", len, duration_elapsed(&bench));
  return true;
}

bool ProgramBIN::targetExcuteImage(uint32_t base_address, uint32_t len, const char* buffer) {
  int retval = ERROR_OK;
  struct duration bench;
  duration_start(&bench);
  retval = target_write_buffer(current_target_, base_address, len, (uint8_t*)buffer);
  if (retval != ERROR_OK)
    return false;
  duration_measure(&bench);
  //command_print("Program %x size Block image spend %fs", len, duration_elapsed(&bench));
  return true;
}

//check board status 
bool ProgramBIN::checkFSBLBootStatus() {
  uint32_t buff[10][4] = {0};
  struct target *target = get_current_target(command_context::cmd_ctx());
  for (auto i = 1; static_cast<TargetRegister>(i) <= kIO_PLL_CTRL_REG; i++) {
    CHECK_RET(targetReadRegister(static_cast<TargetRegister>(i), buff[i]));
  }
  
  if (*buff[kPLL_CFG_REG] != 0x000FA220) {
    command_print("PLL_CFG initial regisiter failed");
    return false;
  }
  if (*buff[kPLL_CTRL_REG] != 0x00028008) {
    command_print("PLL_CFG initial regisiter failed");
    return false;
  }
  if (*buff[kCLK_CTRL_REG] != 0x1F000200) {
    command_print("CLK_CTRL initial regisiter failed");
    return false;
  }
  if (*buff[kIO_PLL_CFG_REG] != 0x001452C0) {
    command_print("IO_PLL_CFG initial regisiter failed");
    return false;
  }
  if (*buff[kIO_PLL_CTRL_REG] != 0x0001E008) {
    command_print("IO_PLL_CTRL initial regisiter failed");
    return false;
  }
  return true;
}
bool ProgramBIN::clearRemapMemory() {
  uint32_t value = 0;
  int retval = targetReadRegister(kOCM_CFG_REG, &value);
  if (value & 0x1F == 0x1F) {
    targetWriteRegister(kOCM_ENABLE_REG, 0x0000DF0D);
    targetWriteRegister(kOCM_CFG_REG, 0x00000018);
    targetWriteRegister(kOCM_CTRL_REG, 0x0000767B);
  }
  return true;
}

bool ProgramBIN::detectBootloaderPath(const std::string& flash_type, std::string& bootload_path) {
#ifdef WIN32
  const char back_slash = '\\';
  const char* back_slash_str = "\\";
#else
  const char back_slash = '/';
  const char* back_slash_str = "/";
#endif
  std::string path = std::string(JtagGetDataDir());
  bootload_path = path + std::string(back_slash_str) + "zynqflash" + 
    std::string(back_slash_str) + flash_type + ".bin";
  return true;
}

bool ProgramBIN::setBootloaderStatus() {
  bool retval = true;
  uint32_t value = 0;
  //remapping ocm memory for bootloader
  do {
    CHECK_RET(targetReadRegister(kOCM_ENABLE_REG, &value));
    if (value == 0) {
      CHECK_RET(targetWriteRegister(kOCM_ENABLE_REG, 0x0000DF0D));
    }
    CHECK_RET(targetWriteRegister(kOCM_CFG_REG, 0x000001FF));
    CHECK_RET(targetReadRegister(kOCM_CFG_REG, &value));
    if (value & 0x1F != 0x1F) {    
      retval = false;
      break;
    }
    CHECK_RET(targetReadRegister(kOCM_CTRL_REG, &value));
    if (value == 0) {
      CHECK_RET(targetWriteRegister(kOCM_CTRL_REG, 0x0000767B));
    }
  } while (0);
  if (!retval) {
    command_print("OCM remapp memory failed");
  }
  return retval;
}

bool ProgramBIN::executeBinFile() {
  int retval = 0;
  bool bvalue = false;
  char buf[100] = {0};
  int Bin_count = 0;
  int read_bytes = 0;
  uint32_t section_len = 0;
  struct duration bench;
  int section_count = 0;
  uint32_t block_size = 0x20000;
  long start_addr = 0x0;
  BootloaderConsole *jtagconsole;
  jtagconsole = BootloaderConsole::instance();
  duration_start(&bench);
  //command_run_line("start_ocm");
#if 1
  if (current_target_->state != TARGET_RUNNING) {
    targetResume(-1);
  }
  struct target *target = get_current_target(command_context::cmd_ctx());
  //BIN addr  bit_cnt
  if (!OpenocdServer::recvMsg(buf, sizeof(buf) - 1, &read_bytes)) {
    command_print("Read failed in executeProgram!");
    return false;
  }
  setAdapterSpeed(6000);
  buf[read_bytes] = 0;
  std::vector<std::string> bin_command_section = SplitStringWithDelim(buf, ' ');
  if (bin_command_section[0] != "BIN")
    return false;
  start_addr = 0xfffc0000;// std::stoul(bin_command_section[1], nullptr, 16);;
  Bin_count = stoi(bin_command_section[2], nullptr, 10);
  int load_addr = 0;
  char erase_temp[100];
  sprintf(erase_temp, "sf erase 0 0x%x", Bin_count); //----FIXME each flash page erase 64kb
  std::string erase_string("sf erase 0 0xD00000");
  //jtagconsole->setStreamOutMode(BootloaderConsole::DCC_SDKOUT);
  if (!bootload_running_) {
    command_print("bootloader not running in target, please redownload bootloader program");
    return false;
  } else {
    command_print("Performing Erase Operation..."); 
    jtagconsole->dccStreamIn(std::string("sf probe"));
    jtagconsole->dccStreamIn(erase_string);
    command_print("Erase Operation successful...");
  }
  setAdapterSpeed(30000);

  uint8_t* bin_buff = new uint8_t[Bin_count];
  if (!OpenocdServer::recvData(bin_buff, Bin_count)) {
    command_print("Read failed in executeProgram!\n");
    return false;
  }
#endif
#if 0

  std::ofstream test("test_temp_1.bin", std::ofstream::out);
  test.write((char *)bin_buff, Bin_count);
  test.close();
  command_run_line("bin_program test_temp_1.bin");
  delete bin_buff;
#endif
#if 1
  section_count = Bin_count / block_size;
  int last_block_size = Bin_count % block_size;
  if (last_block_size != 0) {
    section_count += 1;
  }
 
  //uint8_t* section_buff = new uint8_t[block_size];
  //start openocd accelerate mode for load_image 
  set_accelerate_mode(true);

  ////halt target
  if (!targetHalt(100)) {
    command_print("target halt failed, please reset board");
    return  false;
  }

  jtagconsole->setStreamOutMode(BootloaderConsole::DCC_NONE);
  size_t offset = 0;
  //BIN addr section_cnt
  for (int cnt = 0; cnt < section_count; cnt++) {
   
    //halt target
    if (!targetHalt(0)) {
      program_failed_ = true;
      return  false;
    }
    section_len = (cnt == section_count - 1 && last_block_size != 0) ? last_block_size : block_size;
    //memcpy(section_buff, bin_buff + cnt * block_size, section_len);
    uint8_t* buffer = bin_buff + offset;
    
    bvalue = targetExcuteImage(target, start_addr, section_len, buffer);
    if (!bvalue) {
      program_failed_ = true;
      break;
    }
    offset += section_len;
    if (cnt == 0)
      command_print_sameline("loading boot image to flash.%%0..");
    if (cnt % 6 == 0)
      command_print_sameline("%%%d..", 10 + (cnt * 90) / section_count);
    if (cnt == section_count-1)
      command_print_sameline("%%100\n");
    if (!targetResume(-1)) {
      command_print("target resume failed, please reset board\n");
      program_failed_ = true;
      return  false;
    }
    char temp[100];
    sprintf(temp, "sf write %x %x %x", 0xFFFC0000, load_addr, section_len);
    load_addr = load_addr + section_len;
    bvalue = BootloaderConsole::instance()->dccStreamIn(std::string(temp));
    if (!bvalue) {
      program_failed_ = true;
      break;
    }
  }

  BootloaderConsole::instance()->setStreamOutMode(BootloaderConsole::DCC_STDOUT);
  //stop openocd accelerate mode for load_image 
  set_accelerate_mode(false);
  setAdapterSpeed(6000);
  command_print("load BOOT image complete");
  delete bin_buff;
#endif
  executecomplete();
  duration_measure(&bench);
  command_print("Program BootImage totally spend %fs", duration_elapsed(&bench));
  return true;
}

bool ProgramBIN::executeElfFile(const std::string& script_initial) {
  char temp[256] = {0};
  setAdapterSpeed(2000);
  CHECK_RET(readBootMode());
  CHECK_RET(clearRemapMemory());
  CHECK_RET(targetHalt(100));
  sprintf(temp, "script %s", script_initial.c_str());
  command_run_line(temp);
  command_run_line("ps7_init");
  executecomplete();
  return true;
}

bool ProgramBIN::executeElfFile() {
  bool bret = false;
  int retval = 0;
  int command = 0;
  char temp[100] = {0};
  int fsbl_len = 0;
  int load_addr = 0;
  bool complete = false;
  //halt target
  setAdapterSpeed(6000);
  CHECK_RET(readBootMode());
  CHECK_RET(clearRemapMemory());
  CHECK_RET(targetHalt(100));
  setAdapterSpeed(30000);
  //start openocd accelerate mode for load_image 
  set_accelerate_mode(true);
  command_print("Running FSBL...");
  //FSBL addr count +++data
  if (!OpenocdServer::recvMsg(temp, sizeof(temp), &command)) {
    command_print("Read failed in executeProgram!\n");
    return false;
  }
  std::vector<std::string> fsbl_section = SplitStringWithDelim(temp, ' ');
  if (fsbl_section[0] != "FSBL") {
    return false;
  }
  load_addr = 0x0;// std::stol(fsbl_section[1], nullptr, 0);
  fsbl_len = std::stoi(fsbl_section[2], nullptr, 10);

  //data
  while (!complete) {
    uint8_t* buf = new uint8_t[fsbl_len + 1];
    int read_bytes = 0;
    if (!OpenocdServer::recvData(buf, fsbl_len)) {
      command_print("Read failed in executeProgram!\n");
      return false;
    }
    bret = targetExcuteImage(current_target_, load_addr, fsbl_len, buf);
    if (!bret) {
      program_failed_ = true;
      break;
    }    
    complete = true;
    delete buf;
  }

  //stop openocd accelerate mode for load_image 
  set_accelerate_mode(false);
  setAdapterSpeed(2000);
  //resume halt targets to initalizition--- FIXME
  command_run_line("targets zynq_ps.cpu0");
  targetResume(0x0);
  targetHalt(100);
  command_run_line("targets zynq_ps.cpu1");
  targetResume(0x0);
  targetHalt(100);
  command_print("Finished running FSBL.");
  executecomplete();
  return true;
}

bool ProgramBIN::executeBootLoader(const std::string& bootloader_path) {
  char temp[1024];
  setAdapterSpeed(2000);
  int boot_addr = 0xfffc0000;
  if (!checkFSBLBootStatus() || !setBootloaderStatus()) {  
    bootload_running_ = false;
    command_print("board initialize failed, please reset board");
    return false;
  }

  CHECK_RET(targetHalt(100));
  setAdapterSpeed(30000);
  sprintf(temp, "load_image %s", bootloader_path.c_str());
  command_run_line(temp);

  if (!targetResume(boot_addr)) {
    bootload_running_ = false;
    return  false;
  }
  command_print("Finished running UBOOT.");
  setAdapterSpeed(6000);
  BootloaderConsole::instance()->dccStreamFlush();
  bootload_running_ = true;
  executecomplete();
  return true;
}

bool ProgramBIN::executeBootLoader() {
  bool retval = false;
  int command = 0;
  char temp[100] = {0};
  bool complete = false;
  uint32_t load_addr = 0;
  uint32_t bootloader_len = 0;
  struct target *target = get_current_target(command_context::cmd_ctx());
  if (!checkFSBLBootStatus() || !setBootloaderStatus()) {
    command_print("board initialize failed, please reset board");
    return false;
  }
  CHECK_RET(targetHalt(100));
  //start openocd accelerate mode for load_image 
  set_accelerate_mode(true);
  //UBOOT addr len +++data
  if (!OpenocdServer::recvMsg(temp, sizeof(temp), &command)) {
    command_print("Read failed in executeProgram!");
    return false;
  }
  std::vector<std::string> bootloader_section = SplitStringWithDelim(temp, ' ');
  if (bootloader_section[0] != "UBOOT") {
    bootload_running_ = false;
    return false;
  }
  load_addr = 0xfffc0000;// std::stol(bootloader_section[1], nullptr, 0);
  bootloader_len = std::stoi(bootloader_section[2], nullptr, 10);
  //data
  while (!complete) {
    uint8_t* buf = new uint8_t[bootloader_len];
    int read_bytes = 0;
    if (!OpenocdServer::recvData(buf, bootloader_len)) {
      bootload_running_ = false;
      command_print("Read failed in executeProgram!");
      return false;
    }
    retval = targetExcuteImage(target, load_addr, bootloader_len, buf);
    if (!retval) {
      bootload_running_ = false;
      break;
    }
    complete = true;
    delete buf;
  }
  if (!targetResume(load_addr)) {
    bootload_running_ = false;
    return  false;
  }

  set_accelerate_mode(false);
  BootloaderConsole::instance()->dccStreamFlush();
  bootload_running_ = true;
  command_print("Finished running UBOOT.");
  executecomplete();
  return true;
}

bool ProgramBIN::executecomplete() {
  //ProgressInfo::updateProgress(100);
  if (!OpenocdServer::sendMsg("program complete\n")) {
    command_print("Write failed!\n");
    return false;
  }
  return true;
}

bool ProgramBIN::readBootMode() {
  uint8_t* buffer = new uint8_t[4];
  CHECK_RET(targetHalt(10));
  uint32_t boot_mode_reg_addr = ZynqInternalRegisterMap[kBOOT_MODE_REG];
  int retval = target_read_memory(current_target_, boot_mode_reg_addr, 4, 1, buffer);
  int status = static_cast<int>(*buffer);
  if (status != 0) {
    command_print("board boot mode not jtag mode, Setting jtag mode to load image file");
    return false;
  }
  delete buffer;
  return true;
}
