//============================================================================
// Copyright (c) 2018, All Right Reserved, FMSH
//
// file:      svf_info.cc
// author:    zhouwei
// purpose:   define the class for svf info
// revision history:
// 
//============================================================================

#include <sstream>
#include <fstream>
#include "server/openocd_server.h"
#include "svf/svf.h"
#include "svf/svf_info.h"
#include "jtag/jtag.h"
#include "target/target.h"
#include "helper/time_support.h"
#include "transport/transport.h"
#include "urjtag/cmd.h"

ExecuteSvf::ProcessTdoData ExecuteSvf::process_tdo_data_ = kTdoNoCare;
bool ExecuteSvf::tdo_match_flag_ = false;
int ExecuteSvf::tdo_data_len_ = 0;
int ExecuteSvf::progress_percent_ = 0;
uint8_t* ExecuteSvf::tdo_data_ = NULL;
std::string DetectPart::error_info("DetectPart Failed:");
bool DetectPart::interface_configed = false;
const char* JtagCmds[] = { 
  "exit", "detect", "svf_command", "select_cable", "connect_cable", "disconnect_cable",
  "set_log_file", "recv_progress", "set_svn_version", "program_image"
};

const char* JtagOneCmds[] {
  "init_state", "select_part", "flush_cable", "cmd_complete", "SIR", "SIR_TDO_DATA", "SIR_TDO_FLAG",
  "SDR", "SDR_TDO_DATA", "SDR_TDO_FLAG", "TRST", "ENDIR", "ENDDR", "STATE", "RUNTEST", "sdr_bit",
  "sdr_get_waves", "sdr_big", "sdr_big_tdo_flag", "sdr_big_tdo_data"
};

CableInterfaceType DetectPart::current_interface_ = kUnknowCable;

std::vector<std::string> SplitStringWithDelim(const std::string &s,
                                              char delim) {
  std::string sTmp(s);
  size_t stringLocation;

  // remove '\r' and '\n'
  while((stringLocation=sTmp.find("\r"))!=std::string::npos)
    sTmp.replace(stringLocation,1,"");
  while((stringLocation=sTmp.find("\t"))!=std::string::npos)
    sTmp.replace(stringLocation,1," ");
  while((stringLocation=sTmp.find("\n"))!=std::string::npos)
    sTmp.replace(stringLocation,1," ");
  std::stringstream ss(sTmp);
  std::string item;
  std::vector<std::string> elems;
  while(std::getline(ss, item, delim)) {
    //skip the empty
    if(1) {
      if (item.empty() == true) {
        if (elems.size() == 0 ||
          elems.back().empty() == true) {
          elems.push_back(item); // push back empty
        }
        continue;
      }
    }
    elems.push_back(item);
  }
  return elems;
}

//class ExecuteSvf
void ExecuteSvf::executeInitState() {
  if (OpenocdServer::cable_type() == kXilinxJtagCable) {
    jtag_svf_cmd_.executeInitState();
  } else {
    SvfCommand* svf_cmd = SvfCommand::svf_cmd();
    svf_cmd->cmdTRST("OFF");
    svf_cmd->cmdENDIR("IDLE");
    svf_cmd->cmdENDDR("IDLE");
    svf_cmd->cmdSTATE("RESET");
    svf_cmd->cmdSTATE("IDLE");
    // FIXME Freq
  }
}

void ExecuteSvf::executeHxrTxrSelPart(int sel_part) {
  Jtag_ChainInfo* chain_info = Jtag_ChainInfo::instance();
  auto chains = chain_info->chains();
  int hir_len = 0, tir_len = 0;
  int hdr_num = sel_part;
  int part_total = static_cast<int>(chains.size());
  int tdr_num = part_total - sel_part - 1;
  if (sel_part < part_total) {
    for (int i = 0; i < sel_part; i++) {
      hir_len += chains[i]->ir_length();
    }
    for (int i = sel_part + 1; i < part_total; i++) {
      tir_len += chains[i]->ir_length();
    }
  }  else {
    tir_len = tdr_num = hir_len = hdr_num = 0;
  }
  if (OpenocdServer::cable_type() == kXilinxJtagCable) {
    jtag_svf_cmd_.cmdTIR(tir_len);
    jtag_svf_cmd_.cmdTDR(tdr_num);
    jtag_svf_cmd_.cmdHIR(hir_len);
    jtag_svf_cmd_.cmdHDR(hdr_num);
  } else {
    SvfCommand::svf_cmd()->cmdTIR(tir_len);
    SvfCommand::svf_cmd()->cmdTDR(tdr_num);
    SvfCommand::svf_cmd()->cmdHIR(hir_len);
    SvfCommand::svf_cmd()->cmdHDR(hdr_num);
  }
  printf("TIR %d; HIR %d; HDR %d; TDR%d\n", tir_len, hir_len, hdr_num, tdr_num);
}
 
bool ExecuteSvf::executeSdrBitstreamCmd(int byte_len) {
  uint32_t* tdi_data = new uint32_t[byte_len/4];
  if (!OpenocdServer::recvData(reinterpret_cast<uint8_t*>(tdi_data), byte_len)) {
    command_print("Read failed!\n");
    return false;
  }
  if (OpenocdServer::cable_type() == kXilinxJtagCable) {
    jtag_svf_cmd_.cmdSdrRawData(byte_len, tdi_data);
  } else {
    SvfCommand::svf_cmd()->cmdSDRBitstream(byte_len, tdi_data);
  }
  return true;
}

bool ExecuteSvf::executeSdrBigCmd(int bit_length, int space_len) {
  command_print("bit-length %d space_length %d\n", bit_length, space_len);
  char* temp_buf = new char[space_len + 20];
  memset(temp_buf, 0, space_len+20);
  int read_bytes = 0;
  if (!OpenocdServer::recvBigMsg(temp_buf, space_len, &read_bytes)) {
    command_print("Read failed in executeProgram!\n");
    return false;
  }
  assert(temp_buf[read_bytes-1] == ';');
  temp_buf[read_bytes-1] = '\0'; // remove last comma
  std::vector<std::string> str_list = SplitStringWithDelim(temp_buf, ' ');
  delete[] temp_buf;
  if (str_list.size() == 1) {
    if (OpenocdServer::cable_type() == kXilinxJtagCable) {
      jtag_svf_cmd_.cmdSDR(bit_length, str_list[0].c_str());
    } else {
      SvfCommand::svf_cmd()->cmdSDR(bit_length, str_list[0]);
    }
  } else if (str_list.size() == 3) {
    if (OpenocdServer::cable_type() == kXilinxJtagCable) {
      jtag_svf_cmd_.cmdSDR(bit_length, str_list[0].c_str(), str_list[1].c_str(), str_list[2].c_str());
    } else {
      SvfCommand::svf_cmd()->cmdSDR(bit_length, str_list[0], str_list[1], str_list[2]);
    }
    processTdoInfo();
  } else {
    assert(0);
    return false;
  }
  return true;
}

bool ExecuteSvf::executeSdrGetWaves(int bits_len) {
  if (OpenocdServer::cable_type() == kXilinxJtagCable) {
    jtag_svf_cmd_.cmdSdrGetWaves(bits_len);
  } else {
    return SvfCommand::svf_cmd()->cmdSDRGetWaves(bits_len);
  }
  return true;
}

bool ExecuteSvf::processTdoInfo() {
  if (process_tdo_data_ == kTdoNoCare) {

  } else if (process_tdo_data_ == kTdoGetFlag) {
    if (OpenocdServer::cable_type() == kXilinxJtagCable) {
      tdo_match_flag_ = jtag_svf_cmd_.tdo_match_flag_;
    }
    sendTdoMatchFlag();
  } else if (process_tdo_data_ == kTdoGetData) {
    if (OpenocdServer::cable_type() == kXilinxJtagCable) {
      set_tdo_data(jtag_svf_cmd_.tdo_data_, jtag_svf_cmd_.bit_len_);
    }
    sendTdoData();

  } else {
    return false;
  }
  return true;
}

void ExecuteSvf::set_tdo_data(uint8_t* src, int bit_len) {
  tdo_data_len_ = (bit_len+7)/8;
  tdo_data_ = src;
}

bool ExecuteSvf::executeOneCmd(std::vector<std::string>& str_list) {
  SvfCommand* svf_cmd = SvfCommand::svf_cmd();
  if (!str_list[0].compare(JtagOneCmds[kJtagInitState])) {
    executeInitState();
  } else if (!str_list[0].compare(JtagOneCmds[kJtagSelectPart])) {
    int sel_part = atoi(str_list[1].c_str());
    executeHxrTxrSelPart(sel_part);
  } else if (!str_list[0].compare(JtagOneCmds[kJtagFlushCable])) {
    if (OpenocdServer::cable_type() == kXilinxJtagCable) {
    } else {
      svf_cmd->flushSvfTap();
    }
  } else if (!str_list[0].compare(0, 3, JtagOneCmds[kJtagSir])) {
    int len = atoi(str_list[1].c_str());
    if (str_list.size() == 3) {
      if (OpenocdServer::cable_type() == kXilinxJtagCable) {
        jtag_svf_cmd_.cmdSIR(len, str_list[2].c_str());
      } else {
        svf_cmd->cmdSIR(len, str_list[2]);
      }
    } else {
      if (!str_list[0].compare(JtagOneCmds[kJtagSirTdoData])) {
        setProcessTdoData(kTdoGetData);
      } else if (!str_list[0].compare(JtagOneCmds[kJtagSirTdoFlag])) {
        setProcessTdoData(kTdoGetFlag);
      } else {
        setProcessTdoData(kTdoNoCare);
      }
      if (OpenocdServer::cable_type() == kXilinxJtagCable) {
        jtag_svf_cmd_.cmdSIR(len, str_list[2].c_str(), str_list[3].c_str(), str_list[4].c_str());
      } else {
        svf_cmd->cmdSIR(len, str_list[2], str_list[3], str_list[4]);
      }
      processTdoInfo();
    }
  } else if (!str_list[0].compare(0, 3, JtagOneCmds[kJtagSdr])) {
    int len = atoi(str_list[1].c_str());
    if (str_list.size() == 3) {
      setProcessTdoData(kNoTdoData);
      if (OpenocdServer::cable_type() == kXilinxJtagCable) {
        jtag_svf_cmd_.cmdSDR(len, str_list[2].c_str());
      } else {
        svf_cmd->cmdSDR(len, str_list[2]);
      }
    } else {
      if (!str_list[0].compare(JtagOneCmds[kJtagSdrTdoData])) {
        setProcessTdoData(kTdoGetData);
      } else if (!str_list[0].compare(JtagOneCmds[kJtagSdrTdoFlag])) {
        setProcessTdoData(kTdoGetFlag);
      } else {
        setProcessTdoData(kTdoNoCare);
      }
      if (OpenocdServer::cable_type() == kXilinxJtagCable) {
        jtag_svf_cmd_.cmdSDR(len, str_list[2].c_str(), str_list[3].c_str(), str_list[4].c_str());
      } else {
        svf_cmd->cmdSDR(len, str_list[2], str_list[3], str_list[4]);
      }
      processTdoInfo();
    }
  } else if (!str_list[0].compare(JtagOneCmds[kJtagSdrBit])) {
    int byte_len = atoi(str_list[1].c_str());
    executeSdrBitstreamCmd(byte_len);
  } else if (!str_list[0].compare(JtagOneCmds[kJtagState]) ||
      !str_list[0].compare(JtagOneCmds[kJtagTrst]) ||
      !str_list[0].compare(JtagOneCmds[kJtagEndIr]) ||
      !str_list[0].compare(JtagOneCmds[kJtagEndDr])) {
    if (!str_list[0].compare(JtagOneCmds[kJtagState])) {
      if (OpenocdServer::cable_type() == kXilinxJtagCable) {
        jtag_svf_cmd_.cmdSTATE(str_list[1].c_str());
      } else {
        svf_cmd->cmdSTATE(str_list[1]);
      }
    } else if (!str_list[0].compare(JtagOneCmds[kJtagTrst])) {
      if (OpenocdServer::cable_type() == kXilinxJtagCable) {
        jtag_svf_cmd_.cmdTRST(str_list[1].c_str());
      } else {
        svf_cmd->cmdTRST(str_list[1]);
      }
    } else if (!str_list[0].compare(JtagOneCmds[kJtagEndIr])) {
      if (OpenocdServer::cable_type() == kXilinxJtagCable) {
        jtag_svf_cmd_.cmdENDIR(str_list[1].c_str());
      } else {
        svf_cmd->cmdENDIR(str_list[1]);
      }
    } else if (!str_list[0].compare(JtagOneCmds[kJtagEndDr])) {
      if (OpenocdServer::cable_type() == kXilinxJtagCable) {
        jtag_svf_cmd_.cmdENDDR(str_list[1].c_str());
      } else {
        svf_cmd->cmdENDDR(str_list[1]);
      }
    }
  } else if (!str_list[0].compare(JtagOneCmds[kJtagRuntest])) {
    if (OpenocdServer::cable_type() == kXilinxJtagCable) {
      if (!str_list[2].compare("TCK")) {
        int count = atoi(str_list[1].c_str());
        jtag_svf_cmd_.cmdRUNTEST(count);
      } else if (!str_list[2].compare("SEC")) {
        int time = atof(str_list[1].c_str()) * 1000; // unit ms
#ifdef WIN32
        Sleep(time);
#else
        sleep(time/1000);
#endif
      }
    } else {
      svf_cmd->cmdRUNTEST(str_list);
    }
  } else if (!str_list[0].compare(0, 7, JtagOneCmds[kJtagSdrBig])) { // sdr_big
    if (!str_list[0].compare(JtagOneCmds[kJtagSdrBig])) {
      setProcessTdoData(kTdoNoCare);
    } else if (!str_list[0].compare(JtagOneCmds[kJtagSdrBigTdoFlag])) {
      setProcessTdoData(kTdoGetFlag);
    } else if (!str_list[0].compare(JtagOneCmds[kJtagSdrBigTdoData])) {
      setProcessTdoData(kTdoGetData);
    }
    int bit_length = atoi(str_list[1].c_str());
    int space_len = atoi(str_list[2].c_str());
    executeSdrBigCmd(bit_length, space_len);
  } else if (!str_list[0].compare(JtagOneCmds[kJtagSdrGetWaves])) {
    setProcessTdoData(kTdoGetData);
    int bits_len = atoi(str_list[1].c_str());
    executeSdrGetWaves(bits_len);
    processTdoInfo();
  } else if (!str_list[0].compare(JtagOneCmds[kJtagCmdComplete])) {
    cmd_complete_ = true;
  } else {
    return false;
  }
  return true;
}

bool ExecuteSvf::program() {
  SvfCommand::svf_cmd()->set_procise_flow(true);
  if (!executeBegin()) {
    command_print("failed in begin!\n");
    return false;
  }
  //procise user manual cancellation when program bit 
  if (program_failed_) 
    return true;
  if (!executeProgram()) {
    command_print("failed in program!\n");
    return false;
  }
  if (program_failed_)
    return true;
  if (!executeEnd()) {
    command_print("failed in end!\n");
    return false;
  }
  if (program_failed_)
    return true;
  SvfCommand::svf_cmd()->set_procise_flow(false);
  return true;
}

bool ExecuteSvf::executeBegin() {
  return true;
}

bool ExecuteSvf::executeProgram() {
  std::string last_cmd_part;
  while (!program_failed_) {
    char buf[16384] = { 0 };
    int read_bytes = 0;
    if (!OpenocdServer::recvMsg(buf, sizeof(buf)-1, &read_bytes)) {
      command_print("Read failed in executeProgram!\n");
      return false;
    }
	buf[read_bytes] = 0;
    std::vector<std::string> cmds = SplitStringWithDelim(buf, ';');
    if (cmds.size() == 0)
      continue;
    cmds[0] = last_cmd_part + cmds[0];
    if (buf[read_bytes-1] == ';') {
      last_cmd_part.clear();
    } else {
      last_cmd_part = cmds[cmds.size()-1];
      cmds.pop_back();
    }
    for (auto& one_cmd : cmds) {
	  int show_size = (one_cmd.size() < 300) ? one_cmd.size() : 300;
	  printf("svf_cmd %s\n", one_cmd.substr(0, show_size).c_str());
      std::vector<std::string> str_list;
      str_list = SplitStringWithDelim(one_cmd, ' ');
      if (!executeOneCmd(str_list)) {
        command_print("error cmd %s in executeProgram!\n", str_list[0].c_str());
		return true;
      }
      if (cmd_complete_ == true) {
        command_print("cmd_complete\n");
        return true;
      }
    }
  }
  return true;
}

bool ExecuteSvf::executeEnd() {
  if (OpenocdServer::cable_type() == kXilinxJtagCable) {
  } else {
    SvfCommand::svf_cmd()->flushSvfTap();
  }
  ProgressInfo::updateProgress(100);
  sendProgramCompleteFlag();
  return true;
}

bool ExecuteSvf::sendProgress(int percent_int) {
  bool send_progress_ = true;
  int kProgressScale = 1;
  if (send_progress_ && (progress_percent_ != percent_int) &&
      (percent_int > 0 && percent_int < 100) && (percent_int % kProgressScale == 0)) {
    progress_percent_ = percent_int;
    char temp[100];
    sprintf(temp, "progress %d\n", percent_int);
    command_print("%s\n", temp);
    if (!OpenocdServer::sendMsg(temp)) {
      command_print("Write failed!\n");
      return false;
    }
  }
  return true;
}

bool ExecuteSvf::sendProgramCompleteFlag() {
  if (!OpenocdServer::sendMsg("program complete\n")) {
    command_print("Write failed!\n");
    return false;
  }
  return true;
}

void ExecuteSvf::setProcessTdoData(ProcessTdoData type) {
  process_tdo_data_ = type;
  jtag_svf_cmd_.process_tdo_data_ =
    (type == kNoTdoData) ? JtagSvfCmd::kNoTdoData :
    (type == kTdoNoCare) ? JtagSvfCmd::kTdoNoCare : 
    (type == kTdoGetFlag) ? JtagSvfCmd::kTdoGetFlag : 
    (type == kTdoGetData) ? JtagSvfCmd::kTdoGetData :
                            JtagSvfCmd::kNoTdoData;
}

bool ExecuteSvf::sendTdoMatchFlag() {
  if (process_tdo_data_ == kTdoGetFlag) {
    if (tdo_match_flag_) {
      if (!OpenocdServer::sendMsg("tdo matched ;")) {
        command_print("Write failed!\n");
        return false;
      }
    } else {
      if (!OpenocdServer::sendMsg("tdo mismatched ;")) {
        command_print("Write failed!\n");
        return false;
      }
    }
  }
  return true;
}
 
bool ExecuteSvf::sendTdoData() {
  if (process_tdo_data_ == kTdoGetData) {
    if (!tdo_data_)
      return false;
    char buf[100];
    sprintf(buf, "tdo_data %d ;", tdo_data_len_);
    if (!OpenocdServer::sendMsg(buf)) {
      command_print("Write failed!\n");
      return false;
    }
    if (!OpenocdServer::sendData(tdo_data_, tdo_data_len_)) {
      command_print("Write failed!\n");
      return false;
    }
  }
  return true;
}

std::string GetJtagAckInfo(JtagCmdType cmd_type) {
  return std::string(JtagCmds[cmd_type]) + " ack";
}

bool JtagExecuteCableCmd() {
  const int test_num = 3;
  bool execute_result = false;
  for (int test_count = 0; test_count < test_num; ++test_count) {
    CmdCable cmd_cable;
    char *params[3] = { "cable", "xpc_ext", NULL };
    execute_result = static_cast<bool>(cmd_cable.Run(params));  
    if (execute_result == 1) {
      printf("Success connect xilinx cable\n");
      break;
    } else {
      if (test_count < test_num - 1) {
#ifdef WIN32
        Sleep(800);
#endif
        printf("cable sleep!\n");
      } else {
        LOG_DEBUG("Please reset cable\n"); //FIXME
        break;
      }
    }
  }
  return execute_result;
}

bool JtagExecuteCableDisconnect() {
  Cable* cable = Cable::cable();
  cable->CableDisconnect();
  return true;
}

bool JtagExecuteDetect() {
  CmdDetect cmd_detect;
  char *params[] = { "detect", NULL };
  bool execute_result = static_cast<bool>(cmd_detect.Run(params));
  if (execute_result == false) {
    LOG_DEBUG("No power up");
  }
  return execute_result;
}

bool DetectPart::sendChainData() {
  char buf[1024];
  char temp[1000];
  char idcode_str[33] = {0};
  std::string part_name;
  std::string manufacturer_name; 
  int part_id = 0, idcode = 0, sir_len = 0;
  bool ret;
  Jtag_ChainInfo * chain_info = Jtag_ChainInfo::instance();
  const std::vector<JtagPart*> chains = chain_info->chains();
  if (chains.size() == 0) {
    if (!OpenocdServer::sendMsg("Cannot open target")) {
      command_print("Write failed!");
      return false;
    }
    detect_failed_ = true;
  }

  memset(buf, 0, sizeof(buf));
  for (size_t cnt = 0; cnt < chains.size(); cnt++) {
    memset(temp, 0, sizeof(temp));
    memset(idcode_str, 0, sizeof(idcode_str));
    part_name  = chains[cnt]->get_part_name();
    manufacturer_name = chains[cnt]->get_manufacturer_name();
    part_id = cnt;
    idcode = chains[cnt]->idcode();
    int2binarystr(idcode,idcode_str);
    idcode_str[32] = '\0';
    sir_len = chains[cnt]->ir_length();
    sprintf(temp,"Device Id: %s PART id: %d IR length: %d Manufacturer: %s Part_name: %s \n ", 
        idcode_str, part_id, sir_len, manufacturer_name.c_str(), part_name.c_str());
    printf("Device Id:\t%s\n PART id :\t%d\n IR length:\t%d\n Manufacturer:\t%s\n Part_name:\t%s\n",
        idcode_str, part_id, sir_len, manufacturer_name.c_str(), part_name.c_str());
    strcat(buf, temp);
  }

  if (!OpenocdServer::sendMsg(buf)) {
    command_print("Write failed");
    return false;
  }

  return true;
}
bool DetectPart::sendErrorMsg() {
  std::string error_msg;
  if (ServerManager::procise_flow_) {
    if (!OpenocdErrorMsg::instance()->getErrorMsg(error_msg)) {
      command_print("Dump error msg failed");
    }
    if (!OpenocdServer::sendMsg(error_msg.c_str())) {
      command_print("Write failed");
      return false;
    }
  }
  command_print("%s", error_msg.c_str());
  detect_failed_ = true;
  return true;
}

bool DetectPart::initChain() {

  if (OpenocdServer::cable_type() == kXilinxJtagCable) {
    auto chain_info = Jtag_ChainInfo::instance();
    chain_info->clearXilinxChainInfo(); // FIXME
    if (!JtagExecuteDetect()) {
      return sendErrorMsg();
    } 
  } else {
    //if (!set_cable_config_info(OpenocdServer::cable_type())) {
    //  command_print("Openocd config interface failue\n");
    //}

    if (!executeBoundaryScan()) {
      return sendErrorMsg();
    }

    if (!configOpenocd()) {
      command_print("openocd device conifgure failue!\n");
      detect_failed_ = true;
    }
  }

  if (ServerManager::procise_flow_) {
    if (!sendChainData()) {
      command_print("Send ChainInfo failed!\n");
      return false;
    }
  }
  
  return true;
}


bool DetectPart::set_cable_config_info(CableInterfaceType type) {

  if (current_interface_ = kUnknowCable) {
    current_interface_ = type;
  } else if (current_interface_ != type) {
    command_print("Can't change the type of interface!\n");
    return false;
  } else {
    return true;
  }

  bool ret = false;
  if (current_interface_ <= kUsbJtagSMT2Cable) {
    USBJTAGConfig config;
    if (current_interface_ == kUsbJtagSMT1Cable) {
      config.usbjtag_info_->updateCableInfo(kCableDevDsc, std::string("Digilent Adept USB Device"));
      config.usbjtag_info_->updateCableInfo(kCablePid, std::string("0x6010"));
    }
    ret = config.execute_interfaceConfig();
  } else if (current_interface_ == kJlinkCable) {
    JlinkConfig config;
    ret = config.execute_interfaceConfig();
  } else if (current_interface_ == kXilinxJtagCable) {
    return true;
  } else {
    command_print("Not specify interface type!\n");
    return false;
  }

  if (!ret) {
    command_print("interface config failue!\n");
    return false;
  }

  DetectPart::interface_configed = true;
  return true;
}

bool DetectPart::configOpenocd() {
  Jtag_ChainInfo * chain_info = Jtag_ChainInfo::instance();
  bool reval = false;
  if (chain_info->chains().size() != 0) {
    OpenocdConfig config;
    reval = config.executeConfig();
  } else {
    command_print("do not detect chain part!\n");
    return false;
  }
  if (!reval) {
    command_print("device config failue!\n");
    return false;
  }
  return true;  
}

bool DetectPart::executeBoundaryScan() {
  Jtag_ChainInfo * chain_info = Jtag_ChainInfo::instance();
  std::vector<uint32_t> old_idcode_vec = chain_info->getIdcodeVec();
  chain_info->clearChainInfo();
  int retval = jtag_adapter_manager::jtag_boundary_scan(command_context::cmd_ctx());
  std::vector<JtagPart*> new_chains = chain_info->chains();
  chain_info->newChainsJudge(old_idcode_vec);
  if (ERROR_OK != retval || new_chains.size() == 0) {
    command_print("Boundary Scan failed");
    OpenocdErrorMsg::instance()->setErrorType(OpenocdErrorMsg::kBoardPowerError);
	  return false;
  }
  //get IR length
  char* path = const_cast<char*>(JtagGetDataDir());
  char  dir[256] = {0};
  strcpy(dir, path);
  if (dir == NULL) {
    command_print("Procise lose Data folder in the installation directory\n");
    return false;
  }
  for (auto cnt = 0; cnt < new_chains.size(); cnt++) {
    detectPartInfo(new_chains[cnt], dir);
    jtag_chain_display(PRINT_LVL_INFO, "Openocd chain_info", "Procise found", new_chains[cnt]);
  }
    
  return true;
}
