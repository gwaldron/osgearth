/*
Copyright 2015 Esri

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

A local copy of the license and additional notices are located with the
source distribution at:

http://github.com/Esri/lerc/

Contributors:  Thomas Maurer
*/

#ifndef LERC2_H
#define LERC2_H

#include <cfloat>
#include <cmath>
#include <algorithm>
#include <string>
#include "BitMask.h"
#include "BitStuffer2.h"

NAMESPACE_LERC_START

/**   Lerc2 v1
 *
 *    -- allow for lossless compression of all common data types
 *    -- avoid data type conversions and copies
 *    -- optimized compression for segmented rasters (10-15x lossless)
 *    -- micro block is 8x8 fixed, only gets doubled to 16x16 if bit rate < 1 bpp
 *    -- cnt is replaced by bit mask
 *    -- Lerc blob header has data range [min, max]
 *    -- harden consistency checks to detect if the byte blob has been tampered with
 *    -- drop support for big endian, this is legacy now
 *
 *    Lerc2 v2
 *
 *    -- add Huffman coding for better lossless compression of 8 bit data types Char, Byte
 *
 *    Lerc2 v3
 *
 *    -- add checksum for the entire byte blob, for more rigorous detection of compressed data corruption
 *    -- for the main bit stuffing routine, use an extra uint buffer for guaranteed memory alignment
 *    -- this also allows to drop the NumExtraBytesToAllocate functions
 *
 *    Lerc2 v4
 *
 *    -- allow array per pixel, nDim values per pixel. Such as RGB, complex number, or larger arrays per pixel
 *    -- extend Huffman coding for 8 bit data types from delta only to trying both delta and orig
 *    -- for integer data types, allow to drop bit planes containing only random noise
 *
 *    Lerc2 v5
 *    -- for float data (as it might be lower precision like %.2f), try raise maxZError if possible w/o extra loss
 *    -- add delta encoding of a block iDim relative to previous block (iDim - 1)
 *
 */

class Lerc2
{
public:
  Lerc2();
  Lerc2(int nDim, int nCols, int nRows, const Byte* pMaskBits = nullptr);    // valid / invalid bits as byte array
  virtual ~Lerc2()  {}

  static int CurrentVersion() { return 5; }

  bool SetEncoderToOldVersion(int version);    // call this to encode compatible to an old decoder

  bool Set(int nDim, int nCols, int nRows, const Byte* pMaskBits = nullptr);

  template<class T>
  unsigned int ComputeNumBytesNeededToWrite(const T* arr, double maxZError, bool encodeMask);

  /// dst buffer already allocated;  byte ptr is moved like a file pointer
  template<class T>
  bool Encode(const T* arr, Byte** ppByte);

  // data types supported by Lerc2
  enum DataType {DT_Char = 0, DT_Byte, DT_Short, DT_UShort, DT_Int, DT_UInt, DT_Float, DT_Double, DT_Undefined};

  struct HeaderInfo
  {
    int version;
    unsigned int checksum;
    int nRows,
        nCols,
        nDim,
        numValidPixel,
        microBlockSize,
        blobSize;

    DataType dt;

    double  maxZError,
            zMin,    // if nDim > 1, this is the overall range
            zMax;

    void RawInit()  { memset(this, 0, sizeof(struct HeaderInfo)); }

    bool TryHuffman() const  { return version > 1 && (dt == DT_Byte || dt == DT_Char) && maxZError == 0.5; }
  };

  static bool GetHeaderInfo(const Byte* pByte, size_t nBytesRemaining, struct HeaderInfo& headerInfo);

  /// dst buffer already allocated;  byte ptr is moved like a file pointer
  template<class T>
  bool Decode(const Byte** ppByte, size_t& nBytesRemaining, T* arr, Byte* pMaskBits = nullptr);    // if mask ptr is not 0, mask bits are returned (even if all valid or same as previous)

private:

  enum ImageEncodeMode { IEM_Tiling = 0, IEM_DeltaHuffman, IEM_Huffman };
  enum BlockEncodeMode { BEM_RawBinary = 0, BEM_BitStuffSimple, BEM_BitStuffLUT };

  int         m_microBlockSize,
              m_maxValToQuantize;
  BitMask     m_bitMask;
  HeaderInfo  m_headerInfo;
  BitStuffer2 m_bitStuffer2;
  bool        m_encodeMask,
              m_writeDataOneSweep;
  ImageEncodeMode  m_imageEncodeMode;

  std::vector<double> m_zMinVec, m_zMaxVec;
  std::vector<std::pair<unsigned short, unsigned int> > m_huffmanCodes;    // <= 256 codes, 1.5 kB

private:
  static std::string FileKey()  { return "Lerc2 "; }
  static bool IsLittleEndianSystem()  { int n = 1;  return (1 == *((Byte*)&n)) && (4 == sizeof(int)); }
  void Init();

  static unsigned int ComputeNumBytesHeaderToWrite(const struct HeaderInfo& hd);
  static bool WriteHeader(Byte** ppByte, const struct HeaderInfo& hd);
  static bool ReadHeader(const Byte** ppByte, size_t& nBytesRemaining, struct HeaderInfo& hd);

  bool WriteMask(Byte** ppByte) const;
  bool ReadMask(const Byte** ppByte, size_t& nBytesRemaining);

  bool DoChecksOnEncode(Byte* pBlobBegin, Byte* pBlobEnd) const;
  static unsigned int ComputeChecksumFletcher32(const Byte* pByte, int len);

  static void AddUIntToCounts(int* pCounts, unsigned int val, int nBits);
  static void AddIntToCounts(int* pCounts, int val, int nBits);

  template<class T>
  bool TryBitPlaneCompression(const T* data, double eps, double& newMaxZError) const;

  template<class T>
  bool TryRaiseMaxZError(const T* data, double& maxZError) const;

  static bool PruneCandidates(std::vector<double>& roundErr, std::vector<double>& zErr,
    std::vector<int>& zFac, double maxZError);

  template<class T>
  bool WriteDataOneSweep(const T* data, Byte** ppByte) const;

  template<class T>
  bool ReadDataOneSweep(const Byte** ppByte, size_t& nBytesRemaining, T* data) const;

  template<class T>
  bool ComputeMinMaxRanges(const T* data, std::vector<double>& zMinVec, std::vector<double>& zMaxVec) const;

  template<class T>
  bool WriteTiles(const T* data, Byte** ppByte, int& numBytes) const;

  template<class T>
  bool ReadTiles(const Byte** ppByte, size_t& nBytesRemaining, T* data) const;

  template<class T>
  bool GetValidDataAndStats(const T* data, int i0, int i1, int j0, int j1, int iDim,
    T* dataBuf, T& zMin, T& zMax, int& numValidPixel, bool& tryLut) const;

  template<class T>
  static bool ComputeDiffSliceInt(const T* data, const T* prevData, int numValidPixel, bool bCheckForIntOverflow,
    double maxZError, std::vector<int>& diffDataVec, int& zMin, int& zMax, bool& tryLut);

  template<class T>
  static bool ComputeDiffSliceFlt(const T* data, const T* prevData, int numValidPixel, bool bCheckForFltRndErr,
    double maxZError, std::vector<T>& diffDataVec, T& zMin, T& zMax, bool& tryLut);

  static bool NeedToCheckForIntOverflow(const HeaderInfo& hd);
  static bool NeedToCheckForFltRndErr(const HeaderInfo& hd);

  static double ComputeMaxVal(double zMin, double zMax, double maxZError);

  template<class T>
  bool NeedToQuantize(int numValidPixel, T zMin, T zMax) const;

  template<class T>
  void Quantize(const T* dataBuf, int num, T zMin, std::vector<unsigned int>& quantVec) const;

  template<class T>
  static void ScaleBack(T* dataBuf, const std::vector<unsigned int>& quantVec,
    double zMin, bool bDiff, bool bClamp, double zMaxClamp, double maxZError);

  template<class T>
  static void ScaleBackConstBlock(T* dataBuf, int num, double zMin, bool bClamp, double zMaxClamp);

  template<class T>
  int NumBytesTile(int numValidPixel, T zMin, T zMax, DataType dtZ, bool tryLut, BlockEncodeMode& blockEncodeMode,
                   const std::vector<std::pair<unsigned int, unsigned int> >& sortedQuantVec) const;

  template<class T>
  bool WriteTile(const T* dataBuf, int num, Byte** ppByte, int& numBytesWritten, int j0, T zMin, T zMax,
    DataType dtZ, bool bDiffEnc, const std::vector<unsigned int>& quantVec, BlockEncodeMode blockEncodeMode,
    const std::vector<std::pair<unsigned int, unsigned int> >& sortedQuantVec) const;

  template<class T>
  bool ReadTile(const Byte** ppByte, size_t& nBytesRemaining, T* data, int i0, int i1, int j0, int j1, int iDim,
                std::vector<unsigned int>& bufferVec) const;

  template<class T>
  static int ReduceDataType(T z, DataType dt, DataType& dtReduced);

  static DataType GetDataTypeUsed(DataType dt, int reducedTypeCode);

  static DataType ValidateDataType(int dt);

  static bool WriteVariableDataType(Byte** ppByte, double z, DataType dtUsed);

  static double ReadVariableDataType(const Byte** ppByte, DataType dtUsed);

  template<class T>
  static DataType GetDataType(T z);

  static unsigned int GetMaxValToQuantize(DataType dt);

  static unsigned int GetDataTypeSize(DataType dt);

  static void SortQuantArray(const std::vector<unsigned int>& quantVec,
    std::vector<std::pair<unsigned int, unsigned int> >& sortedQuantVec);

  template<class T>
  void ComputeHuffmanCodes(const T* data, int& numBytes, ImageEncodeMode& imageEncodeMode,
    std::vector<std::pair<unsigned short, unsigned int> >& codes) const;

  template<class T>
  void ComputeHistoForHuffman(const T* data, std::vector<int>& histo, std::vector<int>& deltaHisto) const;

  template<class T>
  bool EncodeHuffman(const T* data, Byte** ppByte) const;

  template<class T>
  bool DecodeHuffman(const Byte** ppByte, size_t& nBytesRemaining, T* data) const;

  template<class T>
  bool WriteMinMaxRanges(const T* data, Byte** ppByte) const;

  template<class T>
  bool ReadMinMaxRanges(const Byte** ppByte, size_t& nBytesRemaining, const T* data);

  bool CheckMinMaxRanges(bool& minMaxEqual) const;

  template<class T>
  bool FillConstImage(T* data) const;
};

// -------------------------------------------------------------------------- ;
// -------------------------------------------------------------------------- ;

inline
void Lerc2::AddUIntToCounts(int* pCounts, unsigned int val, int nBits)
{
  pCounts[0] += val & 1;
  for (int i = 1; i < nBits; i++)
    pCounts[i] += (val >>= 1) & 1;
}

// -------------------------------------------------------------------------- ;

inline
void Lerc2::AddIntToCounts(int* pCounts, int val, int nBits)
{
  pCounts[0] += val & 1;
  for (int i = 1; i < nBits; i++)
    pCounts[i] += (val >>= 1) & 1;
}

// -------------------------------------------------------------------------- ;

inline bool Lerc2::NeedToCheckForIntOverflow(const HeaderInfo& hd)
{
  return (hd.dt == DT_Int || hd.dt == DT_UInt) && (hd.zMax - hd.zMin >= 0x7FFFFFFF);
}

// -------------------------------------------------------------------------- ;

inline bool Lerc2::NeedToCheckForFltRndErr(const HeaderInfo& hd)
{
  if (hd.dt != DT_Float)
    return false;

  if (hd.zMax - hd.zMin > 100000)  // prevent the below test falls through with extreme numbers like 10^38
    return true;

  float diff = (float)(hd.zMax - hd.zMin);
  double testMax = (double)diff + hd.zMin;
  double fltRndErr = fabs(testMax - hd.zMax);

  return (fltRndErr > hd.maxZError / 8);
}

// -------------------------------------------------------------------------- ;

inline double Lerc2::ComputeMaxVal(double zMin, double zMax, double maxZError)
{
  double fac = 1 / (2 * maxZError);    // must match the code in Decode(), don't touch it
  return (zMax - zMin) * fac;
}

// -------------------------------------------------------------------------- ;

template<class T>
inline bool Lerc2::NeedToQuantize(int numValidPixel, T zMin, T zMax) const
{
  if (numValidPixel == 0 || m_headerInfo.maxZError == 0)
    return false;

  double maxVal = ComputeMaxVal(zMin, zMax, m_headerInfo.maxZError);
  return !(maxVal > m_maxValToQuantize || (unsigned int)(maxVal + 0.5) == 0);
}

// -------------------------------------------------------------------------- ;

template<class T>
inline void Lerc2::Quantize(const T* dataBuf, int num, T zMin, std::vector<unsigned int>& quantVec) const
{
  quantVec.resize(num);

  if (m_headerInfo.dt < DT_Float && m_headerInfo.maxZError == 0.5)    // int lossless
  {
    for (int i = 0; i < num; i++)
      quantVec[i] = (unsigned int)(dataBuf[i] - zMin);    // ok: char, short get promoted to int by C++ integral promotion rule
  }
  else    // float and/or lossy
  {
    double scale = 1 / (2 * m_headerInfo.maxZError);
    double zMinDbl = (double)zMin;

    for (int i = 0; i < num; i++)
      quantVec[i] = (unsigned int)(((double)dataBuf[i] - zMinDbl) * scale + 0.5);    // ok, consistent with ComputeMaxVal(...)
      //quantVec[i] = (unsigned int)((dataBuf[i] - zMin) * scale + 0.5);    // bad, not consistent with ComputeMaxVal(...)
  }
}

// -------------------------------------------------------------------------- ;

template<class T>
inline void Lerc2::ScaleBack(T* dataBuf, const std::vector<unsigned int>& quantVec,
  double zMin, bool bDiff, bool bClamp, double zMaxClamp, double maxZError)
{
  double invScale = 2 * maxZError;    // for int types this is int
  int num = (int)quantVec.size();

  if (!bClamp)
    for (int i = 0; i < num; i++)
    {
      double z = zMin + quantVec[i] * invScale + (bDiff ? dataBuf[i] : 0);
      dataBuf[i] = (T)z;
    }
  else
    for (int i = 0; i < num; i++)
    {
      double z = zMin + quantVec[i] * invScale + (bDiff ? dataBuf[i] : 0);
      dataBuf[i] = (T)std::min(z, zMaxClamp);
    }
}

// -------------------------------------------------------------------------- ;

template<class T>
inline void Lerc2::ScaleBackConstBlock(T* dataBuf, int num, double zMin, bool bClamp, double zMaxClamp)
{
  if (!bClamp)
    for (int i = 0; i < num; i++)
      dataBuf[i] = (T)(zMin + dataBuf[i]);
  else
    for (int i = 0; i < num; i++)
      dataBuf[i] = (T)std::min(zMin + dataBuf[i], zMaxClamp);
}

// -------------------------------------------------------------------------- ;

template<class T>
inline int Lerc2::NumBytesTile(int numValidPixel, T zMin, T zMax, DataType dtZ, bool tryLut,
  BlockEncodeMode& blockEncodeMode, const std::vector<std::pair<unsigned int, unsigned int> >& sortedQuantVec) const
{
  blockEncodeMode = BEM_RawBinary;

  if (numValidPixel == 0 || (zMin == 0 && zMax == 0))
    return 1;

  double maxVal = 0, maxZError = m_headerInfo.maxZError;
  int nBytesRaw = (int)(1 + numValidPixel * sizeof(T));

  if ((maxZError == 0 && zMax > zMin)
    || (maxZError > 0 && (maxVal = ComputeMaxVal(zMin, zMax, maxZError)) > m_maxValToQuantize))
  {
    return nBytesRaw;
  }
  else
  {
    DataType dtReduced;
    ReduceDataType(zMin, dtZ, dtReduced);
    int nBytes = 1 + GetDataTypeSize(dtReduced);

    unsigned int maxElem = (unsigned int)(maxVal + 0.5);
    if (maxElem > 0)
    {
      nBytes += (!tryLut) ? m_bitStuffer2.ComputeNumBytesNeededSimple(numValidPixel, maxElem)
                          : m_bitStuffer2.ComputeNumBytesNeededLut(sortedQuantVec, tryLut);
    }

    if (nBytes < nBytesRaw)
      blockEncodeMode = (!tryLut || maxElem == 0) ? BEM_BitStuffSimple : BEM_BitStuffLUT;
    else
      nBytes = nBytesRaw;

    return nBytes;
  }
}

// -------------------------------------------------------------------------- ;

template<class T>
inline int Lerc2::ReduceDataType(T z, DataType dt, DataType& dtReduced)
{
  Byte b = (Byte)z;
  switch (dt)
  {
    case DT_Short:
    {
      char c = (char)z;
      int tc = (T)c == z ? 2 : (T)b == z ? 1 : 0;
      dtReduced = (DataType)(dt - tc);
      return tc;
    }
    case DT_UShort:
    {
      int tc = (T)b == z ? 1 : 0;
      dtReduced = (DataType)(dt - 2 * tc);
      return tc;
    }
    case DT_Int:
    {
      short s = (short)z;
      unsigned short us = (unsigned short)z;
      int tc = (T)b == z ? 3 : (T)s == z ? 2 : (T)us == z ? 1 : 0;
      dtReduced = (DataType)(dt - tc);
      return tc;
    }
    case DT_UInt:
    {
      unsigned short us = (unsigned short)z;
      int tc = (T)b == z ? 2 : (T)us == z ? 1 : 0;
      dtReduced = (DataType)(dt - 2 * tc);
      return tc;
    }
    case DT_Float:
    {
      short s = (short)z;
      int tc = (T)b == z ? 2 : (T)s == z ? 1 : 0;
      dtReduced = tc == 0 ? dt : (tc == 1 ? DT_Short : DT_Byte);
      return tc;
    }
    case DT_Double:
    {
      short s = (short)z;
      int l = (int)z;
      float f = (float)z;
      int tc = (T)s == z ? 3 : (T)l == z ? 2 : (T)f == z ? 1 : 0;
      dtReduced = tc == 0 ? dt : (DataType)(dt - 2 * tc + 1);
      return tc;
    }
    default:
    {
      dtReduced = dt;
      return 0;
    }
  }
}

// -------------------------------------------------------------------------- ;

inline Lerc2::DataType Lerc2::ValidateDataType(int dt)
{
  if( dt >= DT_Char && dt <= DT_Double )
    return static_cast<DataType>(dt);
  return DT_Undefined;
}

// -------------------------------------------------------------------------- ;

inline
Lerc2::DataType Lerc2::GetDataTypeUsed(DataType dt, int tc)
{
  switch (dt)
  {
    case DT_Short:
    case DT_Int:     return ValidateDataType(dt - tc);
    case DT_UShort:
    case DT_UInt:    return ValidateDataType(dt - 2 * tc);
    case DT_Float:   return tc == 0 ? dt : (tc == 1 ? DT_Short : DT_Byte);
    case DT_Double:  return tc == 0 ? dt : ValidateDataType(dt - 2 * tc + 1);
    default:
      return dt;
  }
}

// -------------------------------------------------------------------------- ;

inline
bool Lerc2::WriteVariableDataType(Byte** ppByte, double z, DataType dtUsed)
{
  Byte* ptr = *ppByte;

  switch (dtUsed)
  {
    case DT_Char:
    {
      *((char*)ptr) = (char)z;
      ptr++;
      break;
    }
    case DT_Byte:
    {
      *((Byte*)ptr) = (Byte)z;
      ptr++;
      break;
    }
    case DT_Short:
    {
      short s = (short)z;
      memcpy(ptr, &s, sizeof(short));
      ptr += 2;
      break;
    }
    case DT_UShort:
    {
      unsigned short us = (unsigned short)z;
      memcpy(ptr, &us, sizeof(unsigned short));
      ptr += 2;
      break;
    }
    case DT_Int:
    {
      int i = (int)z;
      memcpy(ptr, &i, sizeof(int));
      ptr += 4;
      break;
    }
    case DT_UInt:
    {
      unsigned int n = (unsigned int)z;
      memcpy(ptr, &n, sizeof(unsigned int));
      ptr += 4;
      break;
    }
    case DT_Float:
    {
      float f = (float)z;
      memcpy(ptr, &f, sizeof(float));
      ptr += 4;
      break;
    }
    case DT_Double:
    {
      memcpy(ptr, &z, sizeof(double));
      ptr += 8;
      break;
    }

    default:
      return false;
  }

  *ppByte = ptr;
  return true;
}

// -------------------------------------------------------------------------- ;

inline
double Lerc2::ReadVariableDataType(const Byte** ppByte, DataType dtUsed)
{
  const Byte* ptr = *ppByte;

  switch (dtUsed)
  {
    case DT_Char:
    {
      char c = *((char*)ptr);
      *ppByte = ptr + 1;
      return c;
    }
    case DT_Byte:
    {
      Byte b = *((Byte*)ptr);
      *ppByte = ptr + 1;
      return b;
    }
    case DT_Short:
    {
      short s;
      memcpy(&s, ptr, sizeof(short));
      *ppByte = ptr + 2;
      return s;
    }
    case DT_UShort:
    {
      unsigned short us;
      memcpy(&us, ptr, sizeof(unsigned short));
      *ppByte = ptr + 2;
      return us;
    }
    case DT_Int:
    {
      int i;
      memcpy(&i, ptr, sizeof(int));
      *ppByte = ptr + 4;
      return i;
    }
    case DT_UInt:
    {
      unsigned int n;
      memcpy(&n, ptr, sizeof(unsigned int));
      *ppByte = ptr + 4;
      return n;
    }
    case DT_Float:
    {
      float f;
      memcpy(&f, ptr, sizeof(float));
      *ppByte = ptr + 4;
      return f;
    }
    case DT_Double:
    {
      double d;
      memcpy(&d, ptr, sizeof(double));
      *ppByte = ptr + 8;
      return d;
    }
    default:
      return 0;
  }
}

// -------------------------------------------------------------------------- ;

inline
unsigned int Lerc2::GetMaxValToQuantize(DataType dt)
{
  switch (dt)
  {
    case DT_Char:
    case DT_Byte:    //return (1 <<  7) - 1;    // disabled: allow LUT mode for 8 bit segmented
    case DT_Short:
    case DT_UShort:  return (1 << 15) - 1;

    case DT_Int:
    case DT_UInt:
    case DT_Float:
    case DT_Double:  return (1 << 30) - 1;

    default:
      return 0;
  }
}

// -------------------------------------------------------------------------- ;

inline
unsigned int Lerc2::GetDataTypeSize(DataType dt)
{
  switch (dt)
  {
    case DT_Char:
    case DT_Byte:   return 1;
    case DT_Short:
    case DT_UShort: return 2;
    case DT_Int:
    case DT_UInt:
    case DT_Float:  return 4;
    case DT_Double: return 8;

    default:
      return 0;
  }
}

// -------------------------------------------------------------------------- ;

inline
bool Lerc2::CheckMinMaxRanges(bool& minMaxEqual) const
{
  int nDim = m_headerInfo.nDim;
  if ((int)m_zMinVec.size() != nDim || (int)m_zMaxVec.size() != nDim)
    return false;

  minMaxEqual = (0 == memcmp(&m_zMinVec[0], &m_zMaxVec[0], nDim * sizeof(m_zMinVec[0])));
  return true;
}

// -------------------------------------------------------------------------- ;

NAMESPACE_LERC_END
#endif
