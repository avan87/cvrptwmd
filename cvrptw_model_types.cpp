/**
 * Autogenerated by Thrift Compiler (0.9.3)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 *  @generated
 */
#include "cvrptw_model_types.h"

#include <algorithm>
#include <ostream>

#include <thrift/TToString.h>




CVRPTWData::~CVRPTWData() throw() {
}


void CVRPTWData::__set_vec(const std::vector<std::vector<int64_t> > & val) {
  this->vec = val;
}

void CVRPTWData::__set_demands(const std::vector<int64_t> & val) {
  this->demands = val;
}

void CVRPTWData::__set_v_caps(const std::vector<int64_t> & val) {
  this->v_caps = val;
}

void CVRPTWData::__set_timeWindows(const std::vector<std::vector<int64_t> > & val) {
  this->timeWindows = val;
}

void CVRPTWData::__set_serviceTime(const std::vector<int64_t> & val) {
  this->serviceTime = val;
}

void CVRPTWData::__set_vehWindows(const std::vector<std::vector<int64_t> > & val) {
  this->vehWindows = val;
}

void CVRPTWData::__set_depots(const std::vector<std::vector<int64_t> > & val) {
  this->depots = val;
}

void CVRPTWData::__set_deliveries(const std::vector<int64_t> & val) {
  this->deliveries = val;
}

void CVRPTWData::__set_pickups(const std::vector<int64_t> & val) {
  this->pickups = val;
}

void CVRPTWData::__set_initialSolutions(const std::vector<std::vector<int64_t> > & val) {
  this->initialSolutions = val;
}

void CVRPTWData::__set_taskType(const std::string& val) {
  this->taskType = val;
}

uint32_t CVRPTWData::read(::apache::thrift::protocol::TProtocol* iprot) {

  apache::thrift::protocol::TInputRecursionTracker tracker(*iprot);
  uint32_t xfer = 0;
  std::string fname;
  ::apache::thrift::protocol::TType ftype;
  int16_t fid;

  xfer += iprot->readStructBegin(fname);

  using ::apache::thrift::protocol::TProtocolException;


  while (true)
  {
    xfer += iprot->readFieldBegin(fname, ftype, fid);
    if (ftype == ::apache::thrift::protocol::T_STOP) {
      break;
    }
    switch (fid)
    {
      case 1:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->vec.clear();
            uint32_t _size0;
            ::apache::thrift::protocol::TType _etype3;
            xfer += iprot->readListBegin(_etype3, _size0);
            this->vec.resize(_size0);
            uint32_t _i4;
            for (_i4 = 0; _i4 < _size0; ++_i4)
            {
              {
                this->vec[_i4].clear();
                uint32_t _size5;
                ::apache::thrift::protocol::TType _etype8;
                xfer += iprot->readListBegin(_etype8, _size5);
                this->vec[_i4].resize(_size5);
                uint32_t _i9;
                for (_i9 = 0; _i9 < _size5; ++_i9)
                {
                  xfer += iprot->readI64(this->vec[_i4][_i9]);
                }
                xfer += iprot->readListEnd();
              }
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.vec = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 2:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->demands.clear();
            uint32_t _size10;
            ::apache::thrift::protocol::TType _etype13;
            xfer += iprot->readListBegin(_etype13, _size10);
            this->demands.resize(_size10);
            uint32_t _i14;
            for (_i14 = 0; _i14 < _size10; ++_i14)
            {
              xfer += iprot->readI64(this->demands[_i14]);
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.demands = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 3:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->v_caps.clear();
            uint32_t _size15;
            ::apache::thrift::protocol::TType _etype18;
            xfer += iprot->readListBegin(_etype18, _size15);
            this->v_caps.resize(_size15);
            uint32_t _i19;
            for (_i19 = 0; _i19 < _size15; ++_i19)
            {
              xfer += iprot->readI64(this->v_caps[_i19]);
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.v_caps = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 4:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->timeWindows.clear();
            uint32_t _size20;
            ::apache::thrift::protocol::TType _etype23;
            xfer += iprot->readListBegin(_etype23, _size20);
            this->timeWindows.resize(_size20);
            uint32_t _i24;
            for (_i24 = 0; _i24 < _size20; ++_i24)
            {
              {
                this->timeWindows[_i24].clear();
                uint32_t _size25;
                ::apache::thrift::protocol::TType _etype28;
                xfer += iprot->readListBegin(_etype28, _size25);
                this->timeWindows[_i24].resize(_size25);
                uint32_t _i29;
                for (_i29 = 0; _i29 < _size25; ++_i29)
                {
                  xfer += iprot->readI64(this->timeWindows[_i24][_i29]);
                }
                xfer += iprot->readListEnd();
              }
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.timeWindows = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 5:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->serviceTime.clear();
            uint32_t _size30;
            ::apache::thrift::protocol::TType _etype33;
            xfer += iprot->readListBegin(_etype33, _size30);
            this->serviceTime.resize(_size30);
            uint32_t _i34;
            for (_i34 = 0; _i34 < _size30; ++_i34)
            {
              xfer += iprot->readI64(this->serviceTime[_i34]);
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.serviceTime = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 6:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->vehWindows.clear();
            uint32_t _size35;
            ::apache::thrift::protocol::TType _etype38;
            xfer += iprot->readListBegin(_etype38, _size35);
            this->vehWindows.resize(_size35);
            uint32_t _i39;
            for (_i39 = 0; _i39 < _size35; ++_i39)
            {
              {
                this->vehWindows[_i39].clear();
                uint32_t _size40;
                ::apache::thrift::protocol::TType _etype43;
                xfer += iprot->readListBegin(_etype43, _size40);
                this->vehWindows[_i39].resize(_size40);
                uint32_t _i44;
                for (_i44 = 0; _i44 < _size40; ++_i44)
                {
                  xfer += iprot->readI64(this->vehWindows[_i39][_i44]);
                }
                xfer += iprot->readListEnd();
              }
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.vehWindows = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 7:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->depots.clear();
            uint32_t _size45;
            ::apache::thrift::protocol::TType _etype48;
            xfer += iprot->readListBegin(_etype48, _size45);
            this->depots.resize(_size45);
            uint32_t _i49;
            for (_i49 = 0; _i49 < _size45; ++_i49)
            {
              {
                this->depots[_i49].clear();
                uint32_t _size50;
                ::apache::thrift::protocol::TType _etype53;
                xfer += iprot->readListBegin(_etype53, _size50);
                this->depots[_i49].resize(_size50);
                uint32_t _i54;
                for (_i54 = 0; _i54 < _size50; ++_i54)
                {
                  xfer += iprot->readI64(this->depots[_i49][_i54]);
                }
                xfer += iprot->readListEnd();
              }
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.depots = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 8:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->deliveries.clear();
            uint32_t _size55;
            ::apache::thrift::protocol::TType _etype58;
            xfer += iprot->readListBegin(_etype58, _size55);
            this->deliveries.resize(_size55);
            uint32_t _i59;
            for (_i59 = 0; _i59 < _size55; ++_i59)
            {
              xfer += iprot->readI64(this->deliveries[_i59]);
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.deliveries = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 9:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->pickups.clear();
            uint32_t _size60;
            ::apache::thrift::protocol::TType _etype63;
            xfer += iprot->readListBegin(_etype63, _size60);
            this->pickups.resize(_size60);
            uint32_t _i64;
            for (_i64 = 0; _i64 < _size60; ++_i64)
            {
              xfer += iprot->readI64(this->pickups[_i64]);
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.pickups = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 10:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->initialSolutions.clear();
            uint32_t _size65;
            ::apache::thrift::protocol::TType _etype68;
            xfer += iprot->readListBegin(_etype68, _size65);
            this->initialSolutions.resize(_size65);
            uint32_t _i69;
            for (_i69 = 0; _i69 < _size65; ++_i69)
            {
              {
                this->initialSolutions[_i69].clear();
                uint32_t _size70;
                ::apache::thrift::protocol::TType _etype73;
                xfer += iprot->readListBegin(_etype73, _size70);
                this->initialSolutions[_i69].resize(_size70);
                uint32_t _i74;
                for (_i74 = 0; _i74 < _size70; ++_i74)
                {
                  xfer += iprot->readI64(this->initialSolutions[_i69][_i74]);
                }
                xfer += iprot->readListEnd();
              }
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.initialSolutions = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 11:
        if (ftype == ::apache::thrift::protocol::T_STRING) {
          xfer += iprot->readString(this->taskType);
          this->__isset.taskType = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      default:
        xfer += iprot->skip(ftype);
        break;
    }
    xfer += iprot->readFieldEnd();
  }

  xfer += iprot->readStructEnd();

  return xfer;
}

uint32_t CVRPTWData::write(::apache::thrift::protocol::TProtocol* oprot) const {
  uint32_t xfer = 0;
  apache::thrift::protocol::TOutputRecursionTracker tracker(*oprot);
  xfer += oprot->writeStructBegin("CVRPTWData");

  xfer += oprot->writeFieldBegin("vec", ::apache::thrift::protocol::T_LIST, 1);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_LIST, static_cast<uint32_t>(this->vec.size()));
    std::vector<std::vector<int64_t> > ::const_iterator _iter75;
    for (_iter75 = this->vec.begin(); _iter75 != this->vec.end(); ++_iter75)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter75).size()));
        std::vector<int64_t> ::const_iterator _iter76;
        for (_iter76 = (*_iter75).begin(); _iter76 != (*_iter75).end(); ++_iter76)
        {
          xfer += oprot->writeI64((*_iter76));
        }
        xfer += oprot->writeListEnd();
      }
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("demands", ::apache::thrift::protocol::T_LIST, 2);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>(this->demands.size()));
    std::vector<int64_t> ::const_iterator _iter77;
    for (_iter77 = this->demands.begin(); _iter77 != this->demands.end(); ++_iter77)
    {
      xfer += oprot->writeI64((*_iter77));
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("v_caps", ::apache::thrift::protocol::T_LIST, 3);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>(this->v_caps.size()));
    std::vector<int64_t> ::const_iterator _iter78;
    for (_iter78 = this->v_caps.begin(); _iter78 != this->v_caps.end(); ++_iter78)
    {
      xfer += oprot->writeI64((*_iter78));
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("timeWindows", ::apache::thrift::protocol::T_LIST, 4);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_LIST, static_cast<uint32_t>(this->timeWindows.size()));
    std::vector<std::vector<int64_t> > ::const_iterator _iter79;
    for (_iter79 = this->timeWindows.begin(); _iter79 != this->timeWindows.end(); ++_iter79)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter79).size()));
        std::vector<int64_t> ::const_iterator _iter80;
        for (_iter80 = (*_iter79).begin(); _iter80 != (*_iter79).end(); ++_iter80)
        {
          xfer += oprot->writeI64((*_iter80));
        }
        xfer += oprot->writeListEnd();
      }
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("serviceTime", ::apache::thrift::protocol::T_LIST, 5);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>(this->serviceTime.size()));
    std::vector<int64_t> ::const_iterator _iter81;
    for (_iter81 = this->serviceTime.begin(); _iter81 != this->serviceTime.end(); ++_iter81)
    {
      xfer += oprot->writeI64((*_iter81));
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("vehWindows", ::apache::thrift::protocol::T_LIST, 6);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_LIST, static_cast<uint32_t>(this->vehWindows.size()));
    std::vector<std::vector<int64_t> > ::const_iterator _iter82;
    for (_iter82 = this->vehWindows.begin(); _iter82 != this->vehWindows.end(); ++_iter82)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter82).size()));
        std::vector<int64_t> ::const_iterator _iter83;
        for (_iter83 = (*_iter82).begin(); _iter83 != (*_iter82).end(); ++_iter83)
        {
          xfer += oprot->writeI64((*_iter83));
        }
        xfer += oprot->writeListEnd();
      }
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("depots", ::apache::thrift::protocol::T_LIST, 7);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_LIST, static_cast<uint32_t>(this->depots.size()));
    std::vector<std::vector<int64_t> > ::const_iterator _iter84;
    for (_iter84 = this->depots.begin(); _iter84 != this->depots.end(); ++_iter84)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter84).size()));
        std::vector<int64_t> ::const_iterator _iter85;
        for (_iter85 = (*_iter84).begin(); _iter85 != (*_iter84).end(); ++_iter85)
        {
          xfer += oprot->writeI64((*_iter85));
        }
        xfer += oprot->writeListEnd();
      }
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("deliveries", ::apache::thrift::protocol::T_LIST, 8);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>(this->deliveries.size()));
    std::vector<int64_t> ::const_iterator _iter86;
    for (_iter86 = this->deliveries.begin(); _iter86 != this->deliveries.end(); ++_iter86)
    {
      xfer += oprot->writeI64((*_iter86));
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("pickups", ::apache::thrift::protocol::T_LIST, 9);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>(this->pickups.size()));
    std::vector<int64_t> ::const_iterator _iter87;
    for (_iter87 = this->pickups.begin(); _iter87 != this->pickups.end(); ++_iter87)
    {
      xfer += oprot->writeI64((*_iter87));
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("initialSolutions", ::apache::thrift::protocol::T_LIST, 10);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_LIST, static_cast<uint32_t>(this->initialSolutions.size()));
    std::vector<std::vector<int64_t> > ::const_iterator _iter88;
    for (_iter88 = this->initialSolutions.begin(); _iter88 != this->initialSolutions.end(); ++_iter88)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter88).size()));
        std::vector<int64_t> ::const_iterator _iter89;
        for (_iter89 = (*_iter88).begin(); _iter89 != (*_iter88).end(); ++_iter89)
        {
          xfer += oprot->writeI64((*_iter89));
        }
        xfer += oprot->writeListEnd();
      }
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("taskType", ::apache::thrift::protocol::T_STRING, 11);
  xfer += oprot->writeString(this->taskType);
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldStop();
  xfer += oprot->writeStructEnd();
  return xfer;
}

void swap(CVRPTWData &a, CVRPTWData &b) {
  using ::std::swap;
  swap(a.vec, b.vec);
  swap(a.demands, b.demands);
  swap(a.v_caps, b.v_caps);
  swap(a.timeWindows, b.timeWindows);
  swap(a.serviceTime, b.serviceTime);
  swap(a.vehWindows, b.vehWindows);
  swap(a.depots, b.depots);
  swap(a.deliveries, b.deliveries);
  swap(a.pickups, b.pickups);
  swap(a.initialSolutions, b.initialSolutions);
  swap(a.taskType, b.taskType);
  swap(a.__isset, b.__isset);
}

CVRPTWData::CVRPTWData(const CVRPTWData& other90) {
  vec = other90.vec;
  demands = other90.demands;
  v_caps = other90.v_caps;
  timeWindows = other90.timeWindows;
  serviceTime = other90.serviceTime;
  vehWindows = other90.vehWindows;
  depots = other90.depots;
  deliveries = other90.deliveries;
  pickups = other90.pickups;
  initialSolutions = other90.initialSolutions;
  taskType = other90.taskType;
  __isset = other90.__isset;
}
CVRPTWData& CVRPTWData::operator=(const CVRPTWData& other91) {
  vec = other91.vec;
  demands = other91.demands;
  v_caps = other91.v_caps;
  timeWindows = other91.timeWindows;
  serviceTime = other91.serviceTime;
  vehWindows = other91.vehWindows;
  depots = other91.depots;
  deliveries = other91.deliveries;
  pickups = other91.pickups;
  initialSolutions = other91.initialSolutions;
  taskType = other91.taskType;
  __isset = other91.__isset;
  return *this;
}
void CVRPTWData::printTo(std::ostream& out) const {
  using ::apache::thrift::to_string;
  out << "CVRPTWData(";
  out << "vec=" << to_string(vec);
  out << ", " << "demands=" << to_string(demands);
  out << ", " << "v_caps=" << to_string(v_caps);
  out << ", " << "timeWindows=" << to_string(timeWindows);
  out << ", " << "serviceTime=" << to_string(serviceTime);
  out << ", " << "vehWindows=" << to_string(vehWindows);
  out << ", " << "depots=" << to_string(depots);
  out << ", " << "deliveries=" << to_string(deliveries);
  out << ", " << "pickups=" << to_string(pickups);
  out << ", " << "initialSolutions=" << to_string(initialSolutions);
  out << ", " << "taskType=" << to_string(taskType);
  out << ")";
}


Result::~Result() throw() {
}


void Result::__set_result(const std::vector<std::vector<int64_t> > & val) {
  this->result = val;
}

uint32_t Result::read(::apache::thrift::protocol::TProtocol* iprot) {

  apache::thrift::protocol::TInputRecursionTracker tracker(*iprot);
  uint32_t xfer = 0;
  std::string fname;
  ::apache::thrift::protocol::TType ftype;
  int16_t fid;

  xfer += iprot->readStructBegin(fname);

  using ::apache::thrift::protocol::TProtocolException;


  while (true)
  {
    xfer += iprot->readFieldBegin(fname, ftype, fid);
    if (ftype == ::apache::thrift::protocol::T_STOP) {
      break;
    }
    switch (fid)
    {
      case 1:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->result.clear();
            uint32_t _size92;
            ::apache::thrift::protocol::TType _etype95;
            xfer += iprot->readListBegin(_etype95, _size92);
            this->result.resize(_size92);
            uint32_t _i96;
            for (_i96 = 0; _i96 < _size92; ++_i96)
            {
              {
                this->result[_i96].clear();
                uint32_t _size97;
                ::apache::thrift::protocol::TType _etype100;
                xfer += iprot->readListBegin(_etype100, _size97);
                this->result[_i96].resize(_size97);
                uint32_t _i101;
                for (_i101 = 0; _i101 < _size97; ++_i101)
                {
                  xfer += iprot->readI64(this->result[_i96][_i101]);
                }
                xfer += iprot->readListEnd();
              }
            }
            xfer += iprot->readListEnd();
          }
          this->__isset.result = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      default:
        xfer += iprot->skip(ftype);
        break;
    }
    xfer += iprot->readFieldEnd();
  }

  xfer += iprot->readStructEnd();

  return xfer;
}

uint32_t Result::write(::apache::thrift::protocol::TProtocol* oprot) const {
  uint32_t xfer = 0;
  apache::thrift::protocol::TOutputRecursionTracker tracker(*oprot);
  xfer += oprot->writeStructBegin("Result");

  xfer += oprot->writeFieldBegin("result", ::apache::thrift::protocol::T_LIST, 1);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_LIST, static_cast<uint32_t>(this->result.size()));
    std::vector<std::vector<int64_t> > ::const_iterator _iter102;
    for (_iter102 = this->result.begin(); _iter102 != this->result.end(); ++_iter102)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter102).size()));
        std::vector<int64_t> ::const_iterator _iter103;
        for (_iter103 = (*_iter102).begin(); _iter103 != (*_iter102).end(); ++_iter103)
        {
          xfer += oprot->writeI64((*_iter103));
        }
        xfer += oprot->writeListEnd();
      }
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldStop();
  xfer += oprot->writeStructEnd();
  return xfer;
}

void swap(Result &a, Result &b) {
  using ::std::swap;
  swap(a.result, b.result);
  swap(a.__isset, b.__isset);
}

Result::Result(const Result& other104) {
  result = other104.result;
  __isset = other104.__isset;
}
Result& Result::operator=(const Result& other105) {
  result = other105.result;
  __isset = other105.__isset;
  return *this;
}
void Result::printTo(std::ostream& out) const {
  using ::apache::thrift::to_string;
  out << "Result(";
  out << "result=" << to_string(result);
  out << ")";
}


