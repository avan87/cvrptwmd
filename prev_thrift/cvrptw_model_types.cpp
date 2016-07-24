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
    std::vector<std::vector<int64_t> > ::const_iterator _iter45;
    for (_iter45 = this->vec.begin(); _iter45 != this->vec.end(); ++_iter45)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter45).size()));
        std::vector<int64_t> ::const_iterator _iter46;
        for (_iter46 = (*_iter45).begin(); _iter46 != (*_iter45).end(); ++_iter46)
        {
          xfer += oprot->writeI64((*_iter46));
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
    std::vector<int64_t> ::const_iterator _iter47;
    for (_iter47 = this->demands.begin(); _iter47 != this->demands.end(); ++_iter47)
    {
      xfer += oprot->writeI64((*_iter47));
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("v_caps", ::apache::thrift::protocol::T_LIST, 3);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>(this->v_caps.size()));
    std::vector<int64_t> ::const_iterator _iter48;
    for (_iter48 = this->v_caps.begin(); _iter48 != this->v_caps.end(); ++_iter48)
    {
      xfer += oprot->writeI64((*_iter48));
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("timeWindows", ::apache::thrift::protocol::T_LIST, 4);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_LIST, static_cast<uint32_t>(this->timeWindows.size()));
    std::vector<std::vector<int64_t> > ::const_iterator _iter49;
    for (_iter49 = this->timeWindows.begin(); _iter49 != this->timeWindows.end(); ++_iter49)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter49).size()));
        std::vector<int64_t> ::const_iterator _iter50;
        for (_iter50 = (*_iter49).begin(); _iter50 != (*_iter49).end(); ++_iter50)
        {
          xfer += oprot->writeI64((*_iter50));
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
    std::vector<int64_t> ::const_iterator _iter51;
    for (_iter51 = this->serviceTime.begin(); _iter51 != this->serviceTime.end(); ++_iter51)
    {
      xfer += oprot->writeI64((*_iter51));
    }
    xfer += oprot->writeListEnd();
  }
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("vehWindows", ::apache::thrift::protocol::T_LIST, 6);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_LIST, static_cast<uint32_t>(this->vehWindows.size()));
    std::vector<std::vector<int64_t> > ::const_iterator _iter52;
    for (_iter52 = this->vehWindows.begin(); _iter52 != this->vehWindows.end(); ++_iter52)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter52).size()));
        std::vector<int64_t> ::const_iterator _iter53;
        for (_iter53 = (*_iter52).begin(); _iter53 != (*_iter52).end(); ++_iter53)
        {
          xfer += oprot->writeI64((*_iter53));
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

void swap(CVRPTWData &a, CVRPTWData &b) {
  using ::std::swap;
  swap(a.vec, b.vec);
  swap(a.demands, b.demands);
  swap(a.v_caps, b.v_caps);
  swap(a.timeWindows, b.timeWindows);
  swap(a.serviceTime, b.serviceTime);
  swap(a.vehWindows, b.vehWindows);
  swap(a.__isset, b.__isset);
}

CVRPTWData::CVRPTWData(const CVRPTWData& other54) {
  vec = other54.vec;
  demands = other54.demands;
  v_caps = other54.v_caps;
  timeWindows = other54.timeWindows;
  serviceTime = other54.serviceTime;
  vehWindows = other54.vehWindows;
  __isset = other54.__isset;
}
CVRPTWData& CVRPTWData::operator=(const CVRPTWData& other55) {
  vec = other55.vec;
  demands = other55.demands;
  v_caps = other55.v_caps;
  timeWindows = other55.timeWindows;
  serviceTime = other55.serviceTime;
  vehWindows = other55.vehWindows;
  __isset = other55.__isset;
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
  out << ")";
}


Result::~Result() throw() {
}


void Result::__set_objValue(const int64_t val) {
  this->objValue = val;
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
        if (ftype == ::apache::thrift::protocol::T_I64) {
          xfer += iprot->readI64(this->objValue);
          this->__isset.objValue = true;
        } else {
          xfer += iprot->skip(ftype);
        }
        break;
      case 2:
        if (ftype == ::apache::thrift::protocol::T_LIST) {
          {
            this->result.clear();
            uint32_t _size56;
            ::apache::thrift::protocol::TType _etype59;
            xfer += iprot->readListBegin(_etype59, _size56);
            this->result.resize(_size56);
            uint32_t _i60;
            for (_i60 = 0; _i60 < _size56; ++_i60)
            {
              {
                this->result[_i60].clear();
                uint32_t _size61;
                ::apache::thrift::protocol::TType _etype64;
                xfer += iprot->readListBegin(_etype64, _size61);
                this->result[_i60].resize(_size61);
                uint32_t _i65;
                for (_i65 = 0; _i65 < _size61; ++_i65)
                {
                  xfer += iprot->readI64(this->result[_i60][_i65]);
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

  xfer += oprot->writeFieldBegin("objValue", ::apache::thrift::protocol::T_I64, 1);
  xfer += oprot->writeI64(this->objValue);
  xfer += oprot->writeFieldEnd();

  xfer += oprot->writeFieldBegin("result", ::apache::thrift::protocol::T_LIST, 2);
  {
    xfer += oprot->writeListBegin(::apache::thrift::protocol::T_LIST, static_cast<uint32_t>(this->result.size()));
    std::vector<std::vector<int64_t> > ::const_iterator _iter66;
    for (_iter66 = this->result.begin(); _iter66 != this->result.end(); ++_iter66)
    {
      {
        xfer += oprot->writeListBegin(::apache::thrift::protocol::T_I64, static_cast<uint32_t>((*_iter66).size()));
        std::vector<int64_t> ::const_iterator _iter67;
        for (_iter67 = (*_iter66).begin(); _iter67 != (*_iter66).end(); ++_iter67)
        {
          xfer += oprot->writeI64((*_iter67));
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
  swap(a.objValue, b.objValue);
  swap(a.result, b.result);
  swap(a.__isset, b.__isset);
}

Result::Result(const Result& other68) {
  objValue = other68.objValue;
  result = other68.result;
  __isset = other68.__isset;
}
Result& Result::operator=(const Result& other69) {
  objValue = other69.objValue;
  result = other69.result;
  __isset = other69.__isset;
  return *this;
}
void Result::printTo(std::ostream& out) const {
  using ::apache::thrift::to_string;
  out << "Result(";
  out << "objValue=" << to_string(objValue);
  out << ", " << "result=" << to_string(result);
  out << ")";
}

