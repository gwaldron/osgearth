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

Contributors:   Thomas Maurer
                Lucian Plesea (provided checksum code)
*/

#include <typeinfo>
#include "Defines.h"
#include "Lerc2.h"
#include "Huffman.h"
#include "RLE.h"

USING_NAMESPACE_LERC
using namespace std;

// -------------------------------------------------------------------------- ;

Lerc2::Lerc2()
{
  Init();
}

// -------------------------------------------------------------------------- ;

Lerc2::Lerc2(int nDim, int nCols, int nRows, const Byte* pMaskBits)
{
  Init();
  Set(nDim, nCols, nRows, pMaskBits);
}

// -------------------------------------------------------------------------- ;

bool Lerc2::SetEncoderToOldVersion(int version)
{
  if (version < 2 || version > CurrentVersion())
    return false;

  if (version < 4 && m_headerInfo.nDim > 1)
    return false;

  m_headerInfo.version = version;

  return true;
}

// -------------------------------------------------------------------------- ;

void Lerc2::Init()
{
  m_microBlockSize    = 8;
  m_maxValToQuantize  = 0;
  m_encodeMask        = true;
  m_writeDataOneSweep = false;
  m_imageEncodeMode   = IEM_Tiling;

  m_headerInfo.RawInit();
  m_headerInfo.version = CurrentVersion();
  m_headerInfo.microBlockSize = m_microBlockSize;
}

// -------------------------------------------------------------------------- ;

bool Lerc2::Set(int nDim, int nCols, int nRows, const Byte* pMaskBits)
{
  if (nDim > 1 && m_headerInfo.version < 4)
    return false;

  if (!m_bitMask.SetSize(nCols, nRows))
    return false;

  if (pMaskBits)
  {
    memcpy(m_bitMask.Bits(), pMaskBits, m_bitMask.Size());
    m_headerInfo.numValidPixel = m_bitMask.CountValidBits();
  }
  else
  {
    m_headerInfo.numValidPixel = nCols * nRows;
    m_bitMask.SetAllValid();
  }

  m_headerInfo.nDim  = nDim;
  m_headerInfo.nCols = nCols;
  m_headerInfo.nRows = nRows;

  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
unsigned int Lerc2::ComputeNumBytesNeededToWrite(const T* arr, double maxZError, bool encodeMask)
{
  if (!arr || !IsLittleEndianSystem())
    return 0;

  // header
  unsigned int nBytesHeaderMask = ComputeNumBytesHeaderToWrite(m_headerInfo);

  // valid / invalid mask
  int numValid = m_headerInfo.numValidPixel;
  int numTotal = m_headerInfo.nCols * m_headerInfo.nRows;

  bool needMask = numValid > 0 && numValid < numTotal;

  m_encodeMask = encodeMask;

  nBytesHeaderMask += 1 * sizeof(int);    // the mask encode numBytes

  if (needMask && encodeMask)
  {
    RLE rle;
    size_t n = rle.computeNumBytesRLE((const Byte*)m_bitMask.Bits(), m_bitMask.Size());
    nBytesHeaderMask += (unsigned int)n;
  }

  m_headerInfo.dt = GetDataType(arr[0]);

  if (m_headerInfo.dt == DT_Undefined)
    return 0;

  if (maxZError == 777)    // cheat code
    maxZError = -0.01;

  if (m_headerInfo.dt < DT_Float)    // integer types
  {
    // interpret a negative maxZError as bit plane epsilon; dflt = 0.01;
    if (maxZError < 0 && (!TryBitPlaneCompression(arr, -maxZError, maxZError)))
      maxZError = 0;

    maxZError = std::max(0.5, floor(maxZError));
  }
  else    // float types
  {
    if (maxZError < 0)    // don't allow bit plane compression for float or double
      return 0;

    double maxZErrorNew = maxZError;
    if (TryRaiseMaxZError(arr, maxZErrorNew))
    {
      //printf("%f --> %f\n", maxZError, maxZErrorNew);
      maxZError = maxZErrorNew;
    }
  }

  m_headerInfo.maxZError = maxZError;
  m_headerInfo.zMin = 0;
  m_headerInfo.zMax = 0;
  m_headerInfo.microBlockSize = m_microBlockSize;
  m_headerInfo.blobSize = nBytesHeaderMask;

  if (numValid == 0)
    return nBytesHeaderMask;

  m_maxValToQuantize = GetMaxValToQuantize(m_headerInfo.dt);

  Byte* ptr = nullptr;    // only emulate the writing and just count the bytes needed
  int nBytesTiling = 0;

  if (!ComputeMinMaxRanges(arr, m_zMinVec, m_zMaxVec))    // need this for diff encoding before WriteTiles()
    return 0;

  m_headerInfo.zMin = *std::min_element(m_zMinVec.begin(), m_zMinVec.end());
  m_headerInfo.zMax = *std::max_element(m_zMaxVec.begin(), m_zMaxVec.end());

  if (m_headerInfo.zMin == m_headerInfo.zMax)    // image is const
    return nBytesHeaderMask;

  int nDim = m_headerInfo.nDim;

  if (m_headerInfo.version >= 4)
  {
    // add the min max ranges behind the mask and before the main data;
    // so we do not write it if no valid pixel or all same value const
    m_headerInfo.blobSize += 2 * nDim * sizeof(T);

    bool minMaxEqual = false;
    if (!CheckMinMaxRanges(minMaxEqual))
      return 0;

    if (minMaxEqual)
      return m_headerInfo.blobSize;    // all nDim bands are const
  }

  // data
  if (!WriteTiles(arr, &ptr, nBytesTiling))
    return 0;

  m_imageEncodeMode = IEM_Tiling;
  int nBytesData = nBytesTiling;
  int nBytesHuffman = 0;

  if (m_headerInfo.TryHuffman())
  {
    ImageEncodeMode huffmanEncMode;
    ComputeHuffmanCodes(arr, nBytesHuffman, huffmanEncMode, m_huffmanCodes);    // save Huffman codes for later use

    if (!m_huffmanCodes.empty() && nBytesHuffman < nBytesTiling)
    {
      m_imageEncodeMode = huffmanEncMode;
      nBytesData = nBytesHuffman;
    }
    else
      m_huffmanCodes.resize(0);
  }

  m_writeDataOneSweep = false;
  int nBytesDataOneSweep = (int)(numValid * nDim * sizeof(T));

  {
    // try with double block size to reduce block header overhead, if
    if ((nBytesTiling * 8 < numTotal * nDim * 1.5)    // resulting bit rate < x (2 bpp)
      && (nBytesTiling < 4 * nBytesDataOneSweep)     // bit stuffing is effective
      && (nBytesHuffman == 0 || nBytesTiling < 2 * nBytesHuffman)    // not much worse than huffman (otherwise huffman wins anyway)
      && (m_headerInfo.nRows > m_microBlockSize || m_headerInfo.nCols > m_microBlockSize))
    {
      m_headerInfo.microBlockSize = m_microBlockSize * 2;

      int nBytes2 = 0;
      if (!WriteTiles(arr, &ptr, nBytes2))    // no huffman in here anymore
        return 0;

      if (nBytes2 <= nBytesData)
      {
        nBytesData = nBytes2;
        m_imageEncodeMode = IEM_Tiling;
        m_huffmanCodes.resize(0);
      }
      else
      {
        m_headerInfo.microBlockSize = m_microBlockSize;    // reset to orig
      }
    }
  }

  if (m_headerInfo.TryHuffman())
    nBytesData += 1;    // flag for image encode mode

  if (nBytesDataOneSweep <= nBytesData)
  {
    m_writeDataOneSweep = true;    // fallback: write data binary uncompressed in one sweep
    m_headerInfo.blobSize += 1 + nBytesDataOneSweep;    // header, mask, min max ranges, flag, data one sweep
  }
  else
  {
    m_writeDataOneSweep = false;
    m_headerInfo.blobSize += 1 + nBytesData;    // header, mask, min max ranges, flag(s), data
  }

  return m_headerInfo.blobSize;
}

// -------------------------------------------------------------------------- ;

template unsigned int Lerc2::ComputeNumBytesNeededToWrite<char>(const char* arr, double maxZError, bool encodeMask);
template unsigned int Lerc2::ComputeNumBytesNeededToWrite<Byte>(const Byte* arr, double maxZError, bool encodeMask);
template unsigned int Lerc2::ComputeNumBytesNeededToWrite<short>(const short* arr, double maxZError, bool encodeMask);
template unsigned int Lerc2::ComputeNumBytesNeededToWrite<unsigned short>(const unsigned short* arr, double maxZError, bool encodeMask);
template unsigned int Lerc2::ComputeNumBytesNeededToWrite<int>(const int* arr, double maxZError, bool encodeMask);
template unsigned int Lerc2::ComputeNumBytesNeededToWrite<unsigned int>(const unsigned int* arr, double maxZError, bool encodeMask);
template unsigned int Lerc2::ComputeNumBytesNeededToWrite<float>(const float* arr, double maxZError, bool encodeMask);
template unsigned int Lerc2::ComputeNumBytesNeededToWrite<double>(const double* arr, double maxZError, bool encodeMask);

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::Encode(const T* arr, Byte** ppByte)
{
  if (!arr || !ppByte || !IsLittleEndianSystem())
    return false;

  Byte* ptrBlob = *ppByte;    // keep a ptr to the start of the blob

  if (!WriteHeader(ppByte, m_headerInfo))
    return false;

  if (!WriteMask(ppByte))
    return false;

  if (m_headerInfo.numValidPixel == 0 || m_headerInfo.zMin == m_headerInfo.zMax)
  {
    return DoChecksOnEncode(ptrBlob, *ppByte);
  }

  if (m_headerInfo.version >= 4)
  {
    if (!WriteMinMaxRanges(arr, ppByte))
      return false;

    bool minMaxEqual = false;
    if (!CheckMinMaxRanges(minMaxEqual))
      return false;

    if (minMaxEqual)
      return DoChecksOnEncode(ptrBlob, *ppByte);
  }

  **ppByte = m_writeDataOneSweep ? 1 : 0;    // write flag
  (*ppByte)++;

  if (!m_writeDataOneSweep)
  {
    if (m_headerInfo.TryHuffman())
    {
      **ppByte = (Byte)m_imageEncodeMode;    // Huffman or tiling encode mode
      (*ppByte)++;

      if (!m_huffmanCodes.empty())   // Huffman, no tiling
      {
        if (m_imageEncodeMode != IEM_DeltaHuffman && m_imageEncodeMode != IEM_Huffman)
          return false;

        if (!EncodeHuffman(arr, ppByte))    // data bit stuffed
          return false;

        return DoChecksOnEncode(ptrBlob, *ppByte);
      }
    }

    int numBytes = 0;
    if (!WriteTiles(arr, ppByte, numBytes))
      return false;
  }
  else
  {
    if (!WriteDataOneSweep(arr, ppByte))
      return false;
  }

  return DoChecksOnEncode(ptrBlob, *ppByte);
}

// -------------------------------------------------------------------------- ;

template bool Lerc2::Encode<char>(const char* arr, Byte** ppByte);
template bool Lerc2::Encode<Byte>(const Byte* arr, Byte** ppByte);
template bool Lerc2::Encode<short>(const short* arr, Byte** ppByte);
template bool Lerc2::Encode<unsigned short>(const unsigned short* arr, Byte** ppByte);
template bool Lerc2::Encode<int>(const int* arr, Byte** ppByte);
template bool Lerc2::Encode<unsigned int>(const unsigned int* arr, Byte** ppByte);
template bool Lerc2::Encode<float>(const float* arr, Byte** ppByte);
template bool Lerc2::Encode<double>(const double* arr, Byte** ppByte);

// -------------------------------------------------------------------------- ;

bool Lerc2::GetHeaderInfo(const Byte* pByte, size_t nBytesRemaining, struct HeaderInfo& hd)
{
  if (!pByte || !IsLittleEndianSystem())
    return false;

  return ReadHeader(&pByte, nBytesRemaining, hd);
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::Decode(const Byte** ppByte, size_t& nBytesRemaining, T* arr, Byte* pMaskBits)
{
  if (!arr || !ppByte || !IsLittleEndianSystem())
    return false;

  const Byte* ptrBlob = *ppByte;    // keep a ptr to the start of the blob
  size_t nBytesRemaining00 = nBytesRemaining;

  if (!ReadHeader(ppByte, nBytesRemaining, m_headerInfo))
    return false;

  if (nBytesRemaining00 < (size_t)m_headerInfo.blobSize)
    return false;

  if (m_headerInfo.version >= 3)
  {
    int nBytes = (int)(FileKey().length() + sizeof(int) + sizeof(unsigned int));    // start right after the checksum entry
    if (m_headerInfo.blobSize < nBytes)
      return false;
    unsigned int checksum = ComputeChecksumFletcher32(ptrBlob + nBytes, m_headerInfo.blobSize - nBytes);

    if (checksum != m_headerInfo.checksum)
      return false;
  }

  if (!ReadMask(ppByte, nBytesRemaining))
    return false;

  if (pMaskBits)    // return proper mask bits even if they were not stored
    memcpy(pMaskBits, m_bitMask.Bits(), m_bitMask.Size());

  memset(arr, 0, m_headerInfo.nCols * m_headerInfo.nRows * m_headerInfo.nDim * sizeof(T));

  if (m_headerInfo.numValidPixel == 0)
    return true;

  if (m_headerInfo.zMin == m_headerInfo.zMax)    // image is const
  {
    if (!FillConstImage(arr))
      return false;

    return true;
  }

  if (m_headerInfo.version >= 4)
  {
    if (!ReadMinMaxRanges(ppByte, nBytesRemaining, arr))
      return false;

    bool minMaxEqual = false;
    if (!CheckMinMaxRanges(minMaxEqual))
      return false;

    if (minMaxEqual)    // if all bands are const, fill outgoing and done
    {
      if (!FillConstImage(arr))
        return false;

      return true;    // done
    }
  }

  if (nBytesRemaining < 1)
    return false;

  Byte readDataOneSweep = **ppByte;    // read flag
  (*ppByte)++;
  nBytesRemaining--;

  if (!readDataOneSweep)
  {
    if (m_headerInfo.TryHuffman())
    {
      if (nBytesRemaining < 1)
        return false;

      Byte flag = **ppByte;    // read flag Huffman / Lerc2
      (*ppByte)++;
      nBytesRemaining--;

      if (flag > 2 || (m_headerInfo.version < 4 && flag > 1))
        return false;

      m_imageEncodeMode = (ImageEncodeMode)flag;

      if (m_imageEncodeMode == IEM_DeltaHuffman || m_imageEncodeMode == IEM_Huffman)
      {
        if (!DecodeHuffman(ppByte, nBytesRemaining, arr))
          return false;

        return true;    // done.
      }
    }

    if (!ReadTiles(ppByte, nBytesRemaining, arr))
      return false;
  }
  else
  {
    if (!ReadDataOneSweep(ppByte, nBytesRemaining, arr))
      return false;
  }

  return true;
}

// -------------------------------------------------------------------------- ;

template bool Lerc2::Decode<char>(const Byte** ppByte, size_t& nBytesRemaining, char* arr, Byte* pMaskBits);
template bool Lerc2::Decode<Byte>(const Byte** ppByte, size_t& nBytesRemaining, Byte* arr, Byte* pMaskBits);
template bool Lerc2::Decode<short>(const Byte** ppByte, size_t& nBytesRemaining, short* arr, Byte* pMaskBits);
template bool Lerc2::Decode<unsigned short>(const Byte** ppByte, size_t& nBytesRemaining, unsigned short* arr, Byte* pMaskBits);
template bool Lerc2::Decode<int>(const Byte** ppByte, size_t& nBytesRemaining, int* arr, Byte* pMaskBits);
template bool Lerc2::Decode<unsigned int>(const Byte** ppByte, size_t& nBytesRemaining, unsigned int* arr, Byte* pMaskBits);
template bool Lerc2::Decode<float>(const Byte** ppByte, size_t& nBytesRemaining, float* arr, Byte* pMaskBits);
template bool Lerc2::Decode<double>(const Byte** ppByte, size_t& nBytesRemaining, double* arr, Byte* pMaskBits);

// -------------------------------------------------------------------------- ;
// -------------------------------------------------------------------------- ;

unsigned int Lerc2::ComputeNumBytesHeaderToWrite(const struct HeaderInfo& hd)
{
  unsigned int numBytes = (unsigned int)FileKey().length();
  numBytes += 1 * sizeof(int);
  numBytes += (hd.version >= 3 ? 1 : 0) * sizeof(unsigned int);
  numBytes += (hd.version >= 4 ? 7 : 6) * sizeof(int);
  numBytes += 3 * sizeof(double);
  return numBytes;
}

// -------------------------------------------------------------------------- ;

bool Lerc2::WriteHeader(Byte** ppByte, const struct HeaderInfo& hd)
{
  if (!ppByte)
    return false;

  Byte* ptr = *ppByte;

  string fileKey = FileKey();
  size_t len = fileKey.length();
  memcpy(ptr, fileKey.c_str(), len);
  ptr += len;

  memcpy(ptr, &hd.version, sizeof(int));
  ptr += sizeof(int);

  if (hd.version >= 3)
  {
    unsigned int checksum = 0;
    memcpy(ptr, &checksum, sizeof(unsigned int));    // place holder to be filled by the real check sum later
    ptr += sizeof(unsigned int);
  }

  vector<int> intVec;
  intVec.push_back(hd.nRows);
  intVec.push_back(hd.nCols);

  if (hd.version >= 4)
  {
    intVec.push_back(hd.nDim);
  }

  intVec.push_back(hd.numValidPixel);
  intVec.push_back(hd.microBlockSize);
  intVec.push_back(hd.blobSize);
  intVec.push_back((int)hd.dt);

  len = intVec.size() * sizeof(int);
  memcpy(ptr, &intVec[0], len);
  ptr += len;

  vector<double> dblVec;
  dblVec.push_back(hd.maxZError);
  dblVec.push_back(hd.zMin);
  dblVec.push_back(hd.zMax);

  len = dblVec.size() * sizeof(double);
  memcpy(ptr, &dblVec[0], len);
  ptr += len;

  *ppByte = ptr;
  return true;
}

// -------------------------------------------------------------------------- ;

bool Lerc2::ReadHeader(const Byte** ppByte, size_t& nBytesRemainingInOut, struct HeaderInfo& hd)
{
  if (!ppByte || !*ppByte)
    return false;

  const Byte* ptr = *ppByte;
  size_t nBytesRemaining = nBytesRemainingInOut;

  string fileKey = FileKey();
  size_t keyLen = fileKey.length();

  hd.RawInit();

  if (nBytesRemaining < keyLen || memcmp(ptr, fileKey.c_str(), keyLen))
    return false;

  ptr += keyLen;
  nBytesRemaining -= keyLen;

  if (nBytesRemaining < sizeof(int) || !memcpy(&(hd.version), ptr, sizeof(int)))
    return false;

  ptr += sizeof(int);
  nBytesRemaining -= sizeof(int);

  if (hd.version < 0 || hd.version > CurrentVersion())    // this reader is outdated
    return false;

  if (hd.version >= 3)
  {
    if (nBytesRemaining < sizeof(unsigned int) || !memcpy(&(hd.checksum), ptr, sizeof(unsigned int)))
      return false;

    ptr += sizeof(unsigned int);
    nBytesRemaining -= sizeof(unsigned int);
  }

  int nInts = (hd.version >= 4) ? 7 : 6;
  vector<int> intVec(nInts, 0);
  vector<double> dblVec(3, 0);

  size_t len = sizeof(int) * intVec.size();

  if (nBytesRemaining < len || !memcpy(&intVec[0], ptr, len))
    return false;

  ptr += len;
  nBytesRemaining -= len;

  len = sizeof(double) * dblVec.size();

  if (nBytesRemaining < len || !memcpy(&dblVec[0], ptr, len))
    return false;

  ptr += len;
  nBytesRemaining -= len;

  int i = 0;
  hd.nRows          = intVec[i++];
  hd.nCols          = intVec[i++];
  hd.nDim           = (hd.version >= 4) ? intVec[i++] : 1;
  hd.numValidPixel  = intVec[i++];
  hd.microBlockSize = intVec[i++];
  hd.blobSize       = intVec[i++];
  const int dt      = intVec[i++];
  if (dt < DT_Char || dt > DT_Double)
    return false;
  hd.dt             = static_cast<DataType>(dt);

  hd.maxZError      = dblVec[0];
  hd.zMin           = dblVec[1];
  hd.zMax           = dblVec[2];

  if (hd.nRows <= 0 || hd.nCols <= 0 || hd.nDim <= 0 || hd.numValidPixel < 0 || hd.microBlockSize <= 0 || hd.blobSize <= 0
    || hd.numValidPixel > hd.nRows * hd.nCols)
    return false;

  *ppByte = ptr;
  nBytesRemainingInOut = nBytesRemaining;

  return true;
}

// -------------------------------------------------------------------------- ;

bool Lerc2::WriteMask(Byte** ppByte) const
{
  if (!ppByte)
    return false;

  int numValid = m_headerInfo.numValidPixel;
  int numTotal = m_headerInfo.nCols * m_headerInfo.nRows;

  bool needMask = numValid > 0 && numValid < numTotal;

  Byte* ptr = *ppByte;

  if (needMask && m_encodeMask)
  {
    Byte* pArrRLE;
    size_t numBytesRLE;
    RLE rle;
    if (!rle.compress((const Byte*)m_bitMask.Bits(), m_bitMask.Size(), &pArrRLE, numBytesRLE, false))
      return false;

    int numBytesMask = (int)numBytesRLE;
    memcpy(ptr, &numBytesMask, sizeof(int));    // num bytes for compressed mask
    ptr += sizeof(int);
    memcpy(ptr, pArrRLE, numBytesRLE);
    ptr += numBytesRLE;

    delete[] pArrRLE;
  }
  else
  {
    memset(ptr, 0, sizeof(int));    // indicates no mask stored
    ptr += sizeof(int);
  }

  *ppByte = ptr;
  return true;
}

// -------------------------------------------------------------------------- ;

bool Lerc2::ReadMask(const Byte** ppByte, size_t& nBytesRemainingInOut)
{
  if (!ppByte)
    return false;

  int numValid = m_headerInfo.numValidPixel;
  int w = m_headerInfo.nCols;
  int h = m_headerInfo.nRows;

  const Byte* ptr = *ppByte;
  size_t nBytesRemaining = nBytesRemainingInOut;

  int numBytesMask;
  if (nBytesRemaining < sizeof(int) || !memcpy(&numBytesMask, ptr, sizeof(int)))
    return false;

  ptr += sizeof(int);
  nBytesRemaining -= sizeof(int);

  if (numValid == 0 || numValid == w * h)    // there should be no mask
  {
    if (numBytesMask != 0)
      return false;
  }

  if (!m_bitMask.SetSize(w, h))
    return false;

  if (numValid == 0)
    m_bitMask.SetAllInvalid();
  else if (numValid == w * h)
    m_bitMask.SetAllValid();
  else if (numBytesMask > 0)    // read it in
  {
    if (nBytesRemaining < static_cast<size_t>(numBytesMask))
      return false;

    RLE rle;
    if (!rle.decompress(ptr, nBytesRemaining, m_bitMask.Bits(), m_bitMask.Size()))
      return false;

    ptr += numBytesMask;
    nBytesRemaining -= numBytesMask;
  }
  // else use previous mask

  *ppByte = ptr;
  nBytesRemainingInOut = nBytesRemaining;

  return true;
}

// -------------------------------------------------------------------------- ;

bool Lerc2::DoChecksOnEncode(Byte* pBlobBegin, Byte* pBlobEnd) const
{
  if ((size_t)(pBlobEnd - pBlobBegin) != (size_t)m_headerInfo.blobSize)
    return false;

  if (m_headerInfo.version >= 3)
  {
    int blobSize = (int)(pBlobEnd - pBlobBegin);
    int nBytes = (int)(FileKey().length() + sizeof(int) + sizeof(unsigned int));    // start right after the checksum entry
    if (blobSize < nBytes)
      return false;
    unsigned int checksum = ComputeChecksumFletcher32(pBlobBegin + nBytes, blobSize - nBytes);

    nBytes -= sizeof(unsigned int);
    memcpy(pBlobBegin + nBytes, &checksum, sizeof(unsigned int));
  }

  return true;
}

// -------------------------------------------------------------------------- ;

// from  https://en.wikipedia.org/wiki/Fletcher's_checksum
// modified from ushorts to bytes (by Lucian Plesea)

unsigned int Lerc2::ComputeChecksumFletcher32(const Byte* pByte, int len)
{
  unsigned int sum1 = 0xffff, sum2 = 0xffff;
  unsigned int words = len / 2;

  while (words)
  {
    unsigned int tlen = (words >= 359) ? 359 : words;
    words -= tlen;
    do {
      sum1 += (*pByte++ << 8);
      sum2 += sum1 += *pByte++;
    } while (--tlen);

    sum1 = (sum1 & 0xffff) + (sum1 >> 16);
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  }

  // add the straggler byte if it exists
  if (len & 1)
    sum2 += sum1 += (*pByte << 8);

  // second reduction step to reduce sums to 16 bits
  sum1 = (sum1 & 0xffff) + (sum1 >> 16);
  sum2 = (sum2 & 0xffff) + (sum2 >> 16);

  return sum2 << 16 | sum1;
}


// -------------------------------------------------------------------------- ;

// for the theory and math, see
// https://pdfs.semanticscholar.org/d064/2e2ad1a4c3b445b0d795770f604a5d9e269c.pdf

template<class T>
bool Lerc2::TryBitPlaneCompression(const T* data, double eps, double& newMaxZError) const
{
  newMaxZError = 0;    // lossless is the obvious fallback

  if (!data || eps <= 0)
    return false;

  const HeaderInfo& hd = m_headerInfo;
  const int nDim = hd.nDim;
  const int maxShift = 8 * GetDataTypeSize(hd.dt);
  const int minCnt = 5000;

  if (hd.numValidPixel < minCnt)    // not enough data for good stats
    return false;

  std::vector<int> cntDiffVec(nDim * maxShift, 0);
  int cnt = 0;

  if (nDim == 1 && hd.numValidPixel == hd.nCols * hd.nRows)    // special but common case
  {
    if (hd.dt == DT_Byte || hd.dt == DT_UShort || hd.dt == DT_UInt)    // unsigned int
    {
      for (int i = 0; i < hd.nRows - 1; i++)
        for (int k = i * hd.nCols, j = 0; j < hd.nCols - 1; j++, k++)
        {
          unsigned int c = ((unsigned int)data[k]) ^ ((unsigned int)data[k + 1]);
          AddUIntToCounts(&cntDiffVec[0], c, maxShift);
          cnt++;
          c = ((unsigned int)data[k]) ^ ((unsigned int)data[k + hd.nCols]);
          AddUIntToCounts(&cntDiffVec[0], c, maxShift);
          cnt++;
        }
    }
    else if (hd.dt == DT_Char || hd.dt == DT_Short || hd.dt == DT_Int)    // signed int
    {
      for (int i = 0; i < hd.nRows - 1; i++)
        for (int k = i * hd.nCols, j = 0; j < hd.nCols - 1; j++, k++)
        {
          int c = ((int)data[k]) ^ ((int)data[k + 1]);
          AddIntToCounts(&cntDiffVec[0], c, maxShift);
          cnt++;
          c = ((int)data[k]) ^ ((int)data[k + hd.nCols]);
          AddIntToCounts(&cntDiffVec[0], c, maxShift);
          cnt++;
        }
    }
    else
      return false;    // unsupported data type
  }

  else    // general case:  nDim > 1 or not all pixel valid
  {
    if (hd.dt == DT_Byte || hd.dt == DT_UShort || hd.dt == DT_UInt)    // unsigned int
    {
      for (int k = 0, m0 = 0, i = 0; i < hd.nRows; i++)
        for (int j = 0; j < hd.nCols; j++, k++, m0 += nDim)
          if (m_bitMask.IsValid(k))
          {
            if (j < hd.nCols - 1 && m_bitMask.IsValid(k + 1))    // hori
            {
              for (int s0 = 0, iDim = 0; iDim < nDim; iDim++, s0 += maxShift)
              {
                unsigned int c = ((unsigned int)data[m0 + iDim]) ^ ((unsigned int)data[m0 + iDim + nDim]);
                AddUIntToCounts(&cntDiffVec[s0], c, maxShift);
              }
              cnt++;
            }
            if (i < hd.nRows - 1 && m_bitMask.IsValid(k + hd.nCols))    // vert
            {
              for (int s0 = 0, iDim = 0; iDim < nDim; iDim++, s0 += maxShift)
              {
                unsigned int c = ((unsigned int)data[m0 + iDim]) ^ ((unsigned int)data[m0 + iDim + nDim * hd.nCols]);
                AddUIntToCounts(&cntDiffVec[s0], c, maxShift);
              }
              cnt++;
            }
          }
    }
    else if (hd.dt == DT_Char || hd.dt == DT_Short || hd.dt == DT_Int)    // signed int
    {
      for (int k = 0, m0 = 0, i = 0; i < hd.nRows; i++)
        for (int j = 0; j < hd.nCols; j++, k++, m0 += nDim)
          if (m_bitMask.IsValid(k))
          {
            if (j < hd.nCols - 1 && m_bitMask.IsValid(k + 1))    // hori
            {
              for (int s0 = 0, iDim = 0; iDim < nDim; iDim++, s0 += maxShift)
              {
                int c = ((int)data[m0 + iDim]) ^ ((int)data[m0 + iDim + nDim]);
                AddIntToCounts(&cntDiffVec[s0], c, maxShift);
              }
              cnt++;
            }
            if (i < hd.nRows - 1 && m_bitMask.IsValid(k + hd.nCols))    // vert
            {
              for (int s0 = 0, iDim = 0; iDim < nDim; iDim++, s0 += maxShift)
              {
                int c = ((int)data[m0 + iDim]) ^ ((int)data[m0 + iDim + nDim * hd.nCols]);
                AddIntToCounts(&cntDiffVec[s0], c, maxShift);
              }
              cnt++;
            }
          }
    }
    else
      return false;    // unsupported data type
  }

  if (cnt < minCnt)    // not enough data for good stats
    return false;

  int nCutFound = 0, lastPlaneKept = 0;
  //const bool printAll = false;

  for (int s = maxShift - 1; s >= 0; s--)
  {
    //if (printAll) printf("bit plane %2d: ", s);
    bool bCrit = true;

    for (int iDim = 0; iDim < nDim; iDim++)
    {
      double x = cntDiffVec[iDim * maxShift + s];
      double n = cnt;
      double m = x / n;
      //double stdDev = sqrt(x * x / n - m * m) / n;

      //printf("  %.4f +- %.4f  ", (float)(2 * m), (float)(2 * stdDev));
      //if (printAll) printf("  %.4f ", (float)(2 * m));

      if (fabs(1 - 2 * m) >= eps)
        bCrit = false;
    }
    //if (printAll) printf("\n");

    if (bCrit && nCutFound < 2)
    {
      if (nCutFound == 0)
        lastPlaneKept = s;

      if (nCutFound == 1 && s < lastPlaneKept - 1)
      {
        lastPlaneKept = s;
        nCutFound = 0;
        //if (printAll) printf(" reset ");
      }

      nCutFound++;
      //if (printAll && nCutFound == 1) printf("\n");
    }
  }

  lastPlaneKept = std::max(0, lastPlaneKept);
  //if (printAll) printf("%d \n", lastPlaneKept);

  newMaxZError = (1 << lastPlaneKept) >> 1;    // turn lastPlaneKept into new maxZError

  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::TryRaiseMaxZError(const T* data, double& maxZError) const
{
  if (!data || m_headerInfo.dt < DT_Float || m_headerInfo.numValidPixel == 0)
    return false;

  const HeaderInfo& hd = m_headerInfo;
  const int nDim = hd.nDim;

  std::vector<double> roundErr, zErr, zErrCand = { 1, 0.5, 0.1, 0.05, 0.01, 0.005, 0.001, 0.0005, 0.0001 };
  std::vector<int> zFac, zFacCand = { 1, 2, 10, 20, 100, 200, 1000, 2000, 10000 };

  for (size_t i = 0; i < zErrCand.size(); i++)
    if (zErrCand[i] / 2 > maxZError)
    {
      zErr.push_back(zErrCand[i] / 2);
      zFac.push_back(zFacCand[i]);
      roundErr.push_back(0);
    }

  if (zErr.empty())
    return false;

  if (nDim == 1 && hd.numValidPixel == hd.nCols * hd.nRows)    // special but common case
  {
    for (int i = 0; i < hd.nRows; i++)
    {
      size_t nCand = zErr.size();

      for (int k = i * hd.nCols, j = 0; j < hd.nCols; j++, k++)
      {
        double x = data[k];

        for (size_t n = 0; n < nCand; n++)
        {
          double z = x * zFac[n];
          if (z == (int)z)
            break;

          double delta = fabs(floor(z + 0.5) - z);
          roundErr[n] = std::max(roundErr[n], delta);
        }
      }

      if (!PruneCandidates(roundErr, zErr, zFac, maxZError))
        return false;
    }
  }

  else    // general case:  nDim > 1 or not all pixel valid
  {
    for (int k = 0, m0 = 0, i = 0; i < hd.nRows; i++)
    {
      size_t nCand = zErr.size();

      for (int j = 0; j < hd.nCols; j++, k++, m0 += nDim)
        if (m_bitMask.IsValid(k))
          for (int iDim = 0; iDim < nDim; iDim++)
          {
            double x = data[m0 + iDim];

            for (size_t n = 0; n < nCand; n++)
            {
              double z = x * zFac[n];
              if (z == (int)z)
                break;

              double delta = fabs(floor(z + 0.5) - z);
              roundErr[n] = std::max(roundErr[n], delta);
            }
          }

      if (!PruneCandidates(roundErr, zErr, zFac, maxZError))
        return false;
    }
  }

  for (size_t n = 0; n < zErr.size(); n++)
    if (roundErr[n] / zFac[n] <= maxZError)
    {
      maxZError = zErr[n];
      return true;
    }

  return false;
}

// -------------------------------------------------------------------------- ;

bool Lerc2::PruneCandidates(std::vector<double>& roundErr, std::vector<double>& zErr,
  std::vector<int>& zFac, double maxZError)
{
  size_t nCand = zErr.size();

  if (nCand == 0 || roundErr.size() != nCand || zFac.size() != nCand || maxZError <= 0)
    return false;

  for (int n = (int)(nCand - 1); n >= 0; n--)
    if (roundErr[n] / zFac[n] > maxZError)
    {
      roundErr.erase(roundErr.begin() + n);
      zErr.erase(zErr.begin() + n);
      zFac.erase(zFac.begin() + n);
    }

  return (!zErr.empty());
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::WriteDataOneSweep(const T* data, Byte** ppByte) const
{
  if (!data || !ppByte)
    return false;

  Byte* ptr = (*ppByte);
  const HeaderInfo& hd = m_headerInfo;
  int nDim = hd.nDim;
  int len = nDim * sizeof(T);

  for (int k = 0, m0 = 0, i = 0; i < hd.nRows; i++)
    for (int j = 0; j < hd.nCols; j++, k++, m0 += nDim)
      if (m_bitMask.IsValid(k))
      {
        memcpy(ptr, &data[m0], len);
        ptr += len;
      }

  (*ppByte) = ptr;
  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::ReadDataOneSweep(const Byte** ppByte, size_t& nBytesRemaining, T* data) const
{
  if (!data || !ppByte || !(*ppByte))
    return false;

  const Byte* ptr = (*ppByte);
  const HeaderInfo& hd = m_headerInfo;
  int nDim = hd.nDim;
  int len = nDim * sizeof(T);

  size_t nValidPix = (size_t)m_bitMask.CountValidBits();

  if (nBytesRemaining < nValidPix * len)
    return false;

  for (int k = 0, m0 = 0, i = 0; i < hd.nRows; i++)
    for (int j = 0; j < hd.nCols; j++, k++, m0 += nDim)
      if (m_bitMask.IsValid(k))
      {
        memcpy(&data[m0], ptr, len);
        ptr += len;
      }

  (*ppByte) = ptr;
  nBytesRemaining -= nValidPix * len;

  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::ComputeMinMaxRanges(const T* data, std::vector<double>& zMinVecA, std::vector<double>& zMaxVecA) const
{
  if (!data || m_headerInfo.numValidPixel == 0)
    return false;

  const HeaderInfo& hd = m_headerInfo;
  const int nDim = hd.nDim;
  bool bInit = false;

  zMinVecA.resize(nDim);
  zMaxVecA.resize(nDim);

  std::vector<T> zMinVec(nDim, 0), zMaxVec(nDim, 0);

  if (hd.numValidPixel == hd.nRows * hd.nCols)    // all valid, no mask
  {
    bInit = true;
    for (int iDim = 0; iDim < nDim; iDim++)
      zMinVec[iDim] = zMaxVec[iDim] = data[iDim];

    for (int m0 = 0, i = 0; i < hd.nRows; i++)
      for (int j = 0; j < hd.nCols; j++, m0 += nDim)
        for (int iDim = 0; iDim < nDim; iDim++)
        {
          T val = data[m0 + iDim];

          if (val < zMinVec[iDim])
            zMinVec[iDim] = val;
          else if (val > zMaxVec[iDim])
            zMaxVec[iDim] = val;
        }
  }
  else
  {
    for (int k = 0, m0 = 0, i = 0; i < hd.nRows; i++)
      for (int j = 0; j < hd.nCols; j++, k++, m0 += nDim)
        if (m_bitMask.IsValid(k))
        {
          if (bInit)
            for (int iDim = 0; iDim < nDim; iDim++)
            {
              T val = data[m0 + iDim];

              if (val < zMinVec[iDim])
                zMinVec[iDim] = val;
              else if (val > zMaxVec[iDim])
                zMaxVec[iDim] = val;
            }
          else
          {
            bInit = true;
            for (int iDim = 0; iDim < nDim; iDim++)
              zMinVec[iDim] = zMaxVec[iDim] = data[m0 + iDim];
          }
        }
  }

  if (bInit)
    for (int iDim = 0; iDim < nDim; iDim++)
    {
      zMinVecA[iDim] = zMinVec[iDim];
      zMaxVecA[iDim] = zMaxVec[iDim];
    }

  return bInit;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::WriteTiles(const T* data, Byte** ppByte, int& numBytes) const
{
  if (!data || !ppByte)
    return false;

  numBytes = 0;
  int numBytesLerc = 0;

  std::vector<unsigned int> quantVec, quantVecDiff;
  std::vector<std::pair<unsigned int, unsigned int> > sortedQuantVec, sortedQuantVecDiff;

  const HeaderInfo& hd = m_headerInfo;
  int mbSize = hd.microBlockSize;
  int nDim = hd.nDim;

  std::vector<T> dataVec(mbSize * mbSize, 0);
  T* dataBuf = &dataVec[0];

  const bool bDtInt = (hd.dt < DT_Float);
  const bool bIntLossless = bDtInt && (hd.maxZError == 0.5);
  const bool bTryDiffEnc = (hd.version >= 5) && (nDim > 1);

  const bool bCheckForIntOverflow = NeedToCheckForIntOverflow(hd);
  const bool bCheckForFltRndErr = NeedToCheckForFltRndErr(hd);

  int mbDiff2 = bTryDiffEnc ? mbSize * mbSize : 0;
  std::vector<int> diffDataVecInt(mbDiff2, 0);    // use fixed type (int) for difference of all int types
  std::vector<T> diffDataVecFlt(mbDiff2, 0), prevDataVec(mbDiff2, 0);

  int numTilesVert = (hd.nRows + mbSize - 1) / mbSize;
  int numTilesHori = (hd.nCols + mbSize - 1) / mbSize;

  for (int iTile = 0; iTile < numTilesVert; iTile++)
  {
    int tileH = mbSize;
    int i0 = iTile * tileH;
    if (iTile == numTilesVert - 1)
      tileH = hd.nRows - i0;

    for (int jTile = 0; jTile < numTilesHori; jTile++)
    {
      int tileW = mbSize;
      int j0 = jTile * tileW;
      if (jTile == numTilesHori - 1)
        tileW = hd.nCols - j0;

      for (int iDim = 0; iDim < nDim; iDim++)
      {
        T zMin = 0, zMax = 0;
        int numValidPixel = 0;
        bool bQuantizeDone = false;
        bool tryLut = false;

        if (!GetValidDataAndStats(data, i0, i0 + tileH, j0, j0 + tileW, iDim, dataBuf, zMin, zMax, numValidPixel, tryLut))
          return false;

        if (numValidPixel == 0 && !(*ppByte))
        {
          numBytesLerc += nDim;    // 1 byte per empty block
          break;    // iDim loop
        }

        //tryLut = false;    // always OFF
        //tryLut = NeedToQuantize(numValidPixel, zMin, zMax);    // always ON

        // if needed, quantize the data here once
        if (((*ppByte && iDim == 0) || tryLut) && NeedToQuantize(numValidPixel, zMin, zMax))
        {
          Quantize(dataBuf, numValidPixel, zMin, quantVec);
          bQuantizeDone = true;
          if (tryLut)
            SortQuantArray(quantVec, sortedQuantVec);
        }

        BlockEncodeMode blockEncodeMode(BEM_RawBinary), blockEncodeModeDiff(BEM_RawBinary);
        int numBytesNeeded = NumBytesTile(numValidPixel, zMin, zMax, hd.dt, tryLut, blockEncodeMode, sortedQuantVec);

        int numBytesNeededDiff = numBytesNeeded + 1;
        int zMinDiffInt(0), zMaxDiffInt(0);
        T zMinDiffFlt(0), zMaxDiffFlt(0);
        double zMinDiff(0), zMaxDiff(0);
        bool bQuantizeDoneDiff = false, tryLutDiff = false;

        if (bTryDiffEnc && iDim > 0 && numValidPixel > 0)
        {
          bool rv = bDtInt ? ComputeDiffSliceInt(&dataVec[0], &prevDataVec[0], numValidPixel, bCheckForIntOverflow, hd.maxZError, diffDataVecInt, zMinDiffInt, zMaxDiffInt, tryLutDiff)
            : ComputeDiffSliceFlt(&dataVec[0], &prevDataVec[0], numValidPixel, bCheckForFltRndErr, hd.maxZError, diffDataVecFlt, zMinDiffFlt, zMaxDiffFlt, tryLutDiff);

          zMinDiff = bDtInt ? zMinDiffInt : zMinDiffFlt;
          zMaxDiff = bDtInt ? zMaxDiffInt : zMaxDiffFlt;

          if (rv)    // can be false due to int overflow, then fall back to abs / regular encode
          {
            // tryLutDiff = false;  // for faster encode?

            if (tryLutDiff && NeedToQuantize(numValidPixel, zMinDiff, zMaxDiff))
            {
              bDtInt ? Quantize(&diffDataVecInt[0], numValidPixel, zMinDiffInt, quantVecDiff)
                : Quantize(&diffDataVecFlt[0], numValidPixel, zMinDiffFlt, quantVecDiff);
              bQuantizeDoneDiff = true;
              SortQuantArray(quantVecDiff, sortedQuantVecDiff);
            }

            int nb = bDtInt ? NumBytesTile(numValidPixel, zMinDiffInt, zMaxDiffInt, DT_Int, tryLutDiff, blockEncodeModeDiff, sortedQuantVecDiff)
              : NumBytesTile(numValidPixel, zMinDiffFlt, zMaxDiffFlt, hd.dt, tryLutDiff, blockEncodeModeDiff, sortedQuantVecDiff);
            if (nb > 0)
              numBytesNeededDiff = nb;
          }
        }

        numBytesLerc += std::min(numBytesNeeded, numBytesNeededDiff);

        if (bTryDiffEnc && iDim < (nDim - 1) && numValidPixel > 0)
        {
          if (iDim == 0)
            prevDataVec.resize(numValidPixel);

          if (!bIntLossless)    // if not int lossless: do lossy quantize back and forth, then copy to buffer
          {
            double zMaxClamp = m_zMaxVec[iDim];
            bool bClampScaleBack = (zMax + 2 * hd.maxZError > zMaxClamp);

            if (iDim == 0 || numBytesNeeded <= numBytesNeededDiff)    // for regular or abs encode
            {
              if (bQuantizeDone || NeedToQuantize(numValidPixel, zMin, zMax))
              {
                if (!bQuantizeDone)
                  Quantize(dataBuf, numValidPixel, zMin, quantVec);

                bQuantizeDone = true;
                ScaleBack(&prevDataVec[0], quantVec, zMin, false, bClampScaleBack, zMaxClamp, hd.maxZError);
              }
              else if (zMin == zMax || ((hd.maxZError > 0) && (0 == (unsigned int)(ComputeMaxVal(zMin, zMax, hd.maxZError) + 0.5))))  // const block case
                prevDataVec.assign(numValidPixel, zMin);
              else
                copy(dataVec.begin(), dataVec.begin() + numValidPixel, prevDataVec.begin());
            }
            else    // for diff or relative encode
            {
              if (bQuantizeDoneDiff || NeedToQuantize(numValidPixel, zMinDiff, zMaxDiff))
              {
                if (!bQuantizeDoneDiff)
                  bDtInt ? Quantize(&diffDataVecInt[0], numValidPixel, zMinDiffInt, quantVecDiff)
                  : Quantize(&diffDataVecFlt[0], numValidPixel, zMinDiffFlt, quantVecDiff);

                bQuantizeDoneDiff = true;
                ScaleBack(&prevDataVec[0], quantVecDiff, zMinDiff, true, bClampScaleBack, zMaxClamp, hd.maxZError);
              }
              else if (zMinDiff == zMaxDiff || ((hd.maxZError > 0) && (0 == (unsigned int)(ComputeMaxVal(zMinDiff, zMaxDiff, hd.maxZError) + 0.5))))  // const block case
              {
                ScaleBackConstBlock(&prevDataVec[0], numValidPixel, zMinDiff, bClampScaleBack, zMaxClamp);
              }
              else
                copy(dataVec.begin(), dataVec.begin() + numValidPixel, prevDataVec.begin());
            }
          }
          else
            copy(dataVec.begin(), dataVec.begin() + numValidPixel, prevDataVec.begin());
        }

        if (*ppByte)
        {
          int numBytesWritten = 0;
          bool rv = false;

          if (iDim == 0 || numBytesNeeded <= numBytesNeededDiff)
          {
            if (!bQuantizeDone && NeedToQuantize(numValidPixel, zMin, zMax))
              Quantize(dataBuf, numValidPixel, zMin, quantVec);

            rv = WriteTile(dataBuf, numValidPixel, ppByte, numBytesWritten, j0, zMin, zMax, hd.dt, false, quantVec, blockEncodeMode, sortedQuantVec);
          }
          else
          {
            if (!bQuantizeDoneDiff && NeedToQuantize(numValidPixel, zMinDiff, zMaxDiff))
            {
              bDtInt ? Quantize(&diffDataVecInt[0], numValidPixel, zMinDiffInt, quantVecDiff)
                : Quantize(&diffDataVecFlt[0], numValidPixel, zMinDiffFlt, quantVecDiff);
            }
            rv = bDtInt
              ? WriteTile(&diffDataVecInt[0], numValidPixel, ppByte, numBytesWritten, j0, zMinDiffInt, zMaxDiffInt, DT_Int, true, quantVecDiff, blockEncodeModeDiff, sortedQuantVecDiff)
              : WriteTile(&diffDataVecFlt[0], numValidPixel, ppByte, numBytesWritten, j0, zMinDiffFlt, zMaxDiffFlt, hd.dt, true, quantVecDiff, blockEncodeModeDiff, sortedQuantVecDiff);
          }

          if (!rv || numBytesWritten != std::min(numBytesNeeded, numBytesNeededDiff))
            return false;
        }
      }
    }
  }

  numBytes += numBytesLerc;
  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::ReadTiles(const Byte** ppByte, size_t& nBytesRemaining, T* data) const
{
  if (!data || !ppByte || !(*ppByte))
    return false;

  std::vector<unsigned int> bufferVec;

  const HeaderInfo& hd = m_headerInfo;
  int mbSize = hd.microBlockSize;
  int nDim = hd.nDim;

  if (mbSize > 32)  // fail gracefully in case of corrupted blob for old version <= 2 which had no checksum
    return false;

  int numTilesVert = (hd.nRows + mbSize - 1) / mbSize;
  int numTilesHori = (hd.nCols + mbSize - 1) / mbSize;

  for (int iTile = 0; iTile < numTilesVert; iTile++)
  {
    int tileH = mbSize;
    int i0 = iTile * tileH;
    if (iTile == numTilesVert - 1)
      tileH = hd.nRows - i0;

    for (int jTile = 0; jTile < numTilesHori; jTile++)
    {
      int tileW = mbSize;
      int j0 = jTile * tileW;
      if (jTile == numTilesHori - 1)
        tileW = hd.nCols - j0;

      for (int iDim = 0; iDim < nDim; iDim++)
      {
        if (!ReadTile(ppByte, nBytesRemaining, data, i0, i0 + tileH, j0, j0 + tileW, iDim, bufferVec))
          return false;
      }
    }
  }

  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::GetValidDataAndStats(const T* data, int i0, int i1, int j0, int j1, int iDim,
  T* dataBuf, T& zMin, T& zMax, int& numValidPixel, bool& tryLut) const
{
  const HeaderInfo& hd = m_headerInfo;

  if (!data || i0 < 0 || j0 < 0 || i1 > hd.nRows || j1 > hd.nCols || i0 >= i1 || j0 >= j1 || iDim < 0 || iDim > hd.nDim || !dataBuf)
    return false;

  zMin = zMax = 0;
  tryLut = false;

  T prevVal = 0;
  int cnt = 0, cntSameVal = 0;
  int nDim = hd.nDim;

  if (hd.numValidPixel == hd.nCols * hd.nRows)    // all valid, no mask
  {
    int k0 = i0 * hd.nCols + j0;
    int m0 = k0 * nDim + iDim;
    zMin = zMax = data[m0];    // init

    for (int i = i0; i < i1; i++)
    {
      int k = i * hd.nCols + j0;
      int m = k * nDim + iDim;

      for (int j = j0; j < j1; j++, m += nDim)
      {
        T val = data[m];
        dataBuf[cnt] = val;

        if (val < zMin)
          zMin = val;
        else if (val > zMax)
          zMax = val;

        if (val == prevVal)
          cntSameVal++;

        prevVal = val;
        cnt++;
      }
    }
  }
  else    // not all valid, use mask
  {
    for (int i = i0; i < i1; i++)
    {
      int k = i * hd.nCols + j0;
      int m = k * nDim + iDim;

      for (int j = j0; j < j1; j++, k++, m += nDim)
        if (m_bitMask.IsValid(k))
        {
          T val = data[m];
          dataBuf[cnt] = val;

          if (cnt > 0)
          {
            if (val < zMin)
              zMin = val;
            else if (val > zMax)
              zMax = val;

            if (val == prevVal)
              cntSameVal++;
          }
          else
            zMin = zMax = val;    // init

          prevVal = val;
          cnt++;
        }
    }
  }

  if (cnt > 4)
    tryLut = (zMax > zMin + 3 * hd.maxZError) && (2 * cntSameVal > cnt);

  numValidPixel = cnt;
  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::ComputeDiffSliceInt(const T* data, const T* prevData, int numValidPixel,
  bool bCheckForIntOverflow, double maxZError,
  std::vector<int>& diffDataVec, int& zMin, int& zMax, bool& tryLut)
{
  if (numValidPixel <= 0)
    return false;

  diffDataVec.resize(numValidPixel);

  int prevVal(0), cnt(0), cntSameVal(0);

  if (!bCheckForIntOverflow)
  {
    zMin = zMax = (int)data[0] - (int)prevData[0];    // init

    for (int i = 0; i < numValidPixel; i++)
    {
      int val = (int)data[i] - (int)prevData[i];

      diffDataVec[i] = val;

      if (val < zMin)
        zMin = val;
      else if (val > zMax)
        zMax = val;

      if (val == prevVal)
        cntSameVal++;

      prevVal = val;
      cnt++;
    }
  }
  else
  {
    zMin = zMax = (int)((double)data[0] - (double)prevData[0]);    // init
    const double zIntMax = 0x7FFFFFFF;
    const double zIntMin = -zIntMax - 1;
    bool bOverflow = false;

    for (int i = 0; i < numValidPixel; i++)
    {
      double z = (double)data[i] - (double)prevData[i];
      int val = (int)z;

      if (z < zIntMin || z > zIntMax)    // check for int32 overflow
        bOverflow = true;

      diffDataVec[i] = val;

      if (val < zMin)
        zMin = val;
      else if (val > zMax)
        zMax = val;

      if (val == prevVal)
        cntSameVal++;

      prevVal = val;
      cnt++;
    }

    if (bOverflow)
      return false;
  }

  if (cnt > 4)
    tryLut = (zMax > zMin + 3 * maxZError) && (2 * cntSameVal > cnt);

  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::ComputeDiffSliceFlt(const T* data, const T* prevData, int numValidPixel,
  bool bCheckForFltRndErr, double maxZError,
  std::vector<T>& diffDataVec, T& zMin, T& zMax, bool& tryLut)
{
  if (numValidPixel <= 0)
    return false;

  diffDataVec.resize(numValidPixel);

  zMin = zMax = (T)((double)data[0] - (double)prevData[0]);    // init
  T prevVal(0);
  int cnt(0), cntSameVal(0);

  if (!bCheckForFltRndErr)
  {
    for (int i = 0; i < numValidPixel; i++)
    {
      T val = (T)((double)data[i] - (double)prevData[i]);

      diffDataVec[i] = val;

      if (val < zMin)
        zMin = val;
      else if (val > zMax)
        zMax = val;

      if (val == prevVal)
        cntSameVal++;

      prevVal = val;
      cnt++;
    }
  }
  else
  {
    double maxRoundErr = 0;

    for (int i = 0; i < numValidPixel; i++)
    {
      T val = (T)((double)data[i] - (double)prevData[i]);

      double testVal = (double)val + (double)prevData[i];    // compute flt pnt rounding error
      maxRoundErr = (std::max)(fabs(testVal - (double)data[i]), maxRoundErr);

      diffDataVec[i] = val;

      if (val < zMin)
        zMin = val;
      else if (val > zMax)
        zMax = val;

      if (val == prevVal)
        cntSameVal++;

      prevVal = val;
      cnt++;
    }

    if (maxRoundErr > maxZError / 8)
      return false;
  }

  if (cnt > 4)
    tryLut = (zMax > zMin + 3 * maxZError) && (2 * cntSameVal > cnt);

  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::WriteTile(const T* dataBuf, int num, Byte** ppByte, int& numBytesWritten, int j0, T zMin, T zMax,
  DataType dtZ, bool bDiffEnc, const std::vector<unsigned int>& quantVec, BlockEncodeMode blockEncodeMode,
  const std::vector<std::pair<unsigned int, unsigned int> >& sortedQuantVec) const
{
  Byte* ptr = *ppByte;
  Byte comprFlag = ((j0 >> 3) & 15) << 2;    // use bits 2345 for integrity check

  if (m_headerInfo.version >= 5)
    comprFlag = bDiffEnc ? (comprFlag | 4) : (comprFlag & (7 << 3));    // bit 2 now encodes diff encoding

  if (num == 0 || (zMin == 0 && zMax == 0))    // special cases
  {
    *ptr++ = comprFlag | 2;    // set compression flag to 2 to mark tile as constant 0
    numBytesWritten = 1;
    *ppByte = ptr;
    return true;
  }

  if (blockEncodeMode == BEM_RawBinary)
  {
    if (bDiffEnc)
      return false;    // doesn't make sense, should not happen

    *ptr++ = comprFlag | 0;    // write z's binary uncompressed

    memcpy(ptr, dataBuf, num * sizeof(T));
    ptr += num * sizeof(T);
  }
  else
  {
    double maxVal = (m_headerInfo.maxZError > 0) ? ComputeMaxVal(zMin, zMax, m_headerInfo.maxZError) : 0;

    // write z's as int arr bit stuffed
    unsigned int maxElem = (unsigned int)(maxVal + 0.5);
    if (maxElem == 0)
      comprFlag |= 3;    // set compression flag to 3 to mark tile as constant zMin
    else
      comprFlag |= 1;    // use bit stuffing

    DataType dtReduced;
    int bits67 = ReduceDataType(zMin, dtZ, dtReduced);
    comprFlag |= bits67 << 6;

    *ptr++ = comprFlag;

    if (!WriteVariableDataType(&ptr, (double)zMin, dtReduced))
      return false;

    if (maxElem > 0)
    {
      if ((int)quantVec.size() != num)
        return false;

      if (blockEncodeMode == BEM_BitStuffSimple)
      {
        if (!m_bitStuffer2.EncodeSimple(&ptr, quantVec, m_headerInfo.version))
          return false;
      }
      else if (blockEncodeMode == BEM_BitStuffLUT)
      {
        if (!m_bitStuffer2.EncodeLut(&ptr, sortedQuantVec, m_headerInfo.version))
          return false;
      }
      else
        return false;
    }
  }

  numBytesWritten = (int)(ptr - *ppByte);
  *ppByte = ptr;
  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::ReadTile(const Byte** ppByte, size_t& nBytesRemainingInOut, T* data, int i0, int i1, int j0, int j1, int iDim,
  std::vector<unsigned int>& bufferVec) const
{
  const Byte* ptr = *ppByte;
  size_t nBytesRemaining = nBytesRemainingInOut;

  if (nBytesRemaining < 1)
    return false;

  const HeaderInfo& hd = m_headerInfo;
  int nCols = hd.nCols;
  int nDim = hd.nDim;

  Byte comprFlag = *ptr++;
  nBytesRemaining--;

  const bool bDiffEnc = (hd.version >= 5) ? (comprFlag & 4) : false;  // bit 2 now encodes diff or delta encoding
  const int pattern = (hd.version >= 5) ? 14 : 15;

  if (((comprFlag >> 2) & pattern) != ((j0 >> 3) & pattern))    // use bits 2345 for integrity check
    return false;

  if (bDiffEnc && iDim == 0)
    return false;

  int bits67 = comprFlag >> 6;
  comprFlag &= 3;

  if (comprFlag == 2)    // entire tile is constant 0 (all the valid pixels)
  {
    for (int i = i0; i < i1; i++)
    {
      int k = i * nCols + j0;
      int m = k * nDim + iDim;

      for (int j = j0; j < j1; j++, k++, m += nDim)
        if (m_bitMask.IsValid(k))
          data[m] = bDiffEnc ? data[m - 1] : 0;
    }

    *ppByte = ptr;
    nBytesRemainingInOut = nBytesRemaining;
    return true;
  }

  else if (comprFlag == 0)    // read z's binary uncompressed
  {
    if (bDiffEnc)
      return false;    // doesn't make sense, should not happen

    const T* srcPtr = (const T*)ptr;
    int cnt = 0;

    for (int i = i0; i < i1; i++)
    {
      int k = i * nCols + j0;
      int m = k * nDim + iDim;

      for (int j = j0; j < j1; j++, k++, m += nDim)
        if (m_bitMask.IsValid(k))
        {
          if (nBytesRemaining < sizeof(T))
            return false;

          data[m] = *srcPtr++;
          nBytesRemaining -= sizeof(T);

          cnt++;
        }
    }

    ptr += cnt * sizeof(T);
  }
  else
  {
    // read z's as int arr bit stuffed
    DataType dtUsed = GetDataTypeUsed((bDiffEnc && hd.dt < DT_Float) ? DT_Int : hd.dt, bits67);
    if (dtUsed == DT_Undefined)
      return false;
    size_t n = GetDataTypeSize(dtUsed);
    if (nBytesRemaining < n)
      return false;

    double offset = ReadVariableDataType(&ptr, dtUsed);
    nBytesRemaining -= n;

    double zMax = (hd.version >= 4 && nDim > 1) ? m_zMaxVec[iDim] : hd.zMax;

    if (comprFlag == 3)    // entire tile is constant zMin (all the valid pixels)
    {
      for (int i = i0; i < i1; i++)
      {
        int k = i * nCols + j0;
        int m = k * nDim + iDim;

        if (!bDiffEnc)
        {
          T val = (T)offset;
          for (int j = j0; j < j1; j++, k++, m += nDim)
            if (m_bitMask.IsValid(k))
              data[m] = val;
        }
        else
        {
          for (int j = j0; j < j1; j++, k++, m += nDim)
            if (m_bitMask.IsValid(k))
            {
              double z = offset + data[m - 1];
              data[m] = (T)std::min(z, zMax);
            }
        }
      }
    }
    else
    {
      size_t maxElementCount = (i1 - i0) * (j1 - j0);
      if (!m_bitStuffer2.Decode(&ptr, nBytesRemaining, bufferVec, maxElementCount, hd.version))
        return false;

      double invScale = 2 * hd.maxZError;    // for int types this is int
      const unsigned int* srcPtr = bufferVec.data();

      if (bufferVec.size() == maxElementCount)    // all valid
      {
        for (int i = i0; i < i1; i++)
        {
          int k = i * nCols + j0;
          int m = k * nDim + iDim;

          if (!bDiffEnc)
          {
            for (int j = j0; j < j1; j++, k++, m += nDim)
            {
              double z = offset + *srcPtr++ * invScale;
              data[m] = (T)std::min(z, zMax);    // make sure we stay in the orig range
            }
          }
          else
          {
            for (int j = j0; j < j1; j++, k++, m += nDim)
            {
              double z = offset + *srcPtr++ * invScale + data[m - 1];
              data[m] = (T)std::min(z, zMax);
            }
          }
        }
      }
      else    // not all valid
      {
        if (hd.version > 2)
        {
          for (int i = i0; i < i1; i++)
          {
            int k = i * nCols + j0;
            int m = k * nDim + iDim;

            if (!bDiffEnc)
            {
              for (int j = j0; j < j1; j++, k++, m += nDim)
                if (m_bitMask.IsValid(k))
                {
                  double z = offset + *srcPtr++ * invScale;
                  data[m] = (T)std::min(z, zMax);    // make sure we stay in the orig range
                }
            }
            else
            {
              for (int j = j0; j < j1; j++, k++, m += nDim)
                if (m_bitMask.IsValid(k))
                {
                  double z = offset + *srcPtr++ * invScale + data[m - 1];
                  data[m] = (T)std::min(z, zMax);
                }
            }
          }
        }
        else  // fail gracefully in case of corrupted blob for old version <= 2 which had no checksum
        {
          size_t bufferVecIdx = 0;

          for (int i = i0; i < i1; i++)
          {
            int k = i * nCols + j0;
            int m = k * nDim + iDim;

            for (int j = j0; j < j1; j++, k++, m += nDim)
              if (m_bitMask.IsValid(k))
              {
                if (bufferVecIdx == bufferVec.size())  // fail gracefully in case of corrupted blob for old version <= 2 which had no checksum
                  return false;

                double z = offset + bufferVec[bufferVecIdx] * invScale;
                bufferVecIdx++;
                data[m] = (T)std::min(z, zMax);    // make sure we stay in the orig range
              }
          }
        }
      }
    }
  }

  *ppByte = ptr;
  nBytesRemainingInOut = nBytesRemaining;
  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
Lerc2::DataType Lerc2::GetDataType(T z)
{
  const std::type_info& ti = typeid(z);

  if (ti == typeid(char))            return DT_Char;
  else if (ti == typeid(Byte))            return DT_Byte;
  else if (ti == typeid(short))           return DT_Short;
  else if (ti == typeid(unsigned short))  return DT_UShort;
  else if (ti == typeid(int) && sizeof(int) == 4)   return DT_Int;
  else if (ti == typeid(long) && sizeof(long) == 4)   return DT_Int;
  else if (ti == typeid(unsigned int) && sizeof(unsigned int) == 4)   return DT_UInt;
  else if (ti == typeid(unsigned long) && sizeof(unsigned long) == 4)   return DT_UInt;
  else if (ti == typeid(float))           return DT_Float;
  else if (ti == typeid(double))          return DT_Double;
  else
    return DT_Undefined;
}

// -------------------------------------------------------------------------- ;

void Lerc2::SortQuantArray(const vector<unsigned int>& quantVec, vector<pair<unsigned int, unsigned int> >& sortedQuantVec)
{
  int numElem = (int)quantVec.size();
  sortedQuantVec.resize(numElem);

  for (int i = 0; i < numElem; i++)
    sortedQuantVec[i] = pair<unsigned int, unsigned int>(quantVec[i], i);

  std::sort(sortedQuantVec.begin(), sortedQuantVec.end(),
    [](const pair<unsigned int, unsigned int>& p0,
       const pair<unsigned int, unsigned int>& p1) { return p0.first < p1.first; });
}

// -------------------------------------------------------------------------- ;

template<class T>
void Lerc2::ComputeHuffmanCodes(const T* data, int& numBytes, ImageEncodeMode& imageEncodeMode, std::vector<std::pair<unsigned short, unsigned int> >& codes) const
{
  std::vector<int> histo, deltaHisto;
  ComputeHistoForHuffman(data, histo, deltaHisto);

  int nBytes0 = 0, nBytes1 = 0;
  double avgBpp0 = 0, avgBpp1 = 0;
  Huffman huffman0, huffman1;

  if (m_headerInfo.version >= 4)
  {
    if (!huffman0.ComputeCodes(histo) || !huffman0.ComputeCompressedSize(histo, nBytes0, avgBpp0))
      nBytes0 = 0;
  }

  if (!huffman1.ComputeCodes(deltaHisto) || !huffman1.ComputeCompressedSize(deltaHisto, nBytes1, avgBpp1))
    nBytes1 = 0;

  if (nBytes0 > 0 && nBytes1 > 0)    // regular case, pick the better of the two
  {
    imageEncodeMode = (nBytes0 <= nBytes1) ? IEM_Huffman : IEM_DeltaHuffman;
    codes = (nBytes0 <= nBytes1) ? huffman0.GetCodes() : huffman1.GetCodes();
    numBytes = (std::min)(nBytes0, nBytes1);
  }
  else if (nBytes0 == 0 && nBytes1 == 0)    // rare case huffman cannot handle, fall back to tiling
  {
    imageEncodeMode = IEM_Tiling;
    codes.resize(0);
    numBytes = 0;
  }
  else    // rare also, pick the valid one, the other is 0
  {
    imageEncodeMode = (nBytes0 > nBytes1) ? IEM_Huffman : IEM_DeltaHuffman;
    codes = (nBytes0 > nBytes1) ? huffman0.GetCodes() : huffman1.GetCodes();
    numBytes = (std::max)(nBytes0, nBytes1);
  }
}

// -------------------------------------------------------------------------- ;

template<class T>
void Lerc2::ComputeHistoForHuffman(const T* data, std::vector<int>& histo, std::vector<int>& deltaHisto) const
{
  histo.resize(256);
  deltaHisto.resize(256);

  memset(&histo[0], 0, histo.size() * sizeof(int));
  memset(&deltaHisto[0], 0, deltaHisto.size() * sizeof(int));

  int offset = (m_headerInfo.dt == DT_Char) ? 128 : 0;
  int height = m_headerInfo.nRows;
  int width = m_headerInfo.nCols;
  int nDim = m_headerInfo.nDim;

  if (m_headerInfo.numValidPixel == width * height)    // all valid
  {
    for (int iDim = 0; iDim < nDim; iDim++)
    {
      T prevVal = 0;
      for (int m = iDim, i = 0; i < height; i++)
        for (int j = 0; j < width; j++, m += nDim)
        {
          T val = data[m];
          T delta = val;

          if (j > 0)
            delta -= prevVal;    // use overflow
          else if (i > 0)
            delta -= data[m - width * nDim];
          else
            delta -= prevVal;

          prevVal = val;

          histo[offset + (int)val]++;
          deltaHisto[offset + (int)delta]++;
        }
    }
  }
  else    // not all valid
  {
    for (int iDim = 0; iDim < nDim; iDim++)
    {
      T prevVal = 0;
      for (int k = 0, m = iDim, i = 0; i < height; i++)
        for (int j = 0; j < width; j++, k++, m += nDim)
          if (m_bitMask.IsValid(k))
          {
            T val = data[m];
            T delta = val;

            if (j > 0 && m_bitMask.IsValid(k - 1))
            {
              delta -= prevVal;    // use overflow
            }
            else if (i > 0 && m_bitMask.IsValid(k - width))
            {
              delta -= data[m - width * nDim];
            }
            else
              delta -= prevVal;

            prevVal = val;

            histo[offset + (int)val]++;
            deltaHisto[offset + (int)delta]++;
          }
    }
  }
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::EncodeHuffman(const T* data, Byte** ppByte) const
{
  if (!data || !ppByte)
    return false;

  Huffman huffman;
  if (!huffman.SetCodes(m_huffmanCodes) || !huffman.WriteCodeTable(ppByte, m_headerInfo.version))    // header and code table
    return false;

  int offset = (m_headerInfo.dt == DT_Char) ? 128 : 0;
  int height = m_headerInfo.nRows;
  int width = m_headerInfo.nCols;
  int nDim = m_headerInfo.nDim;

  unsigned int* arr = (unsigned int*)(*ppByte);
  unsigned int* dstPtr = arr;
  int bitPos = 0;

  if (m_imageEncodeMode == IEM_DeltaHuffman)
  {
    for (int iDim = 0; iDim < nDim; iDim++)
    {
      T prevVal = 0;
      for (int k = 0, m = iDim, i = 0; i < height; i++)
        for (int j = 0; j < width; j++, k++, m += nDim)
          if (m_bitMask.IsValid(k))
          {
            T val = data[m];
            T delta = val;

            if (j > 0 && m_bitMask.IsValid(k - 1))
            {
              delta -= prevVal;    // use overflow
            }
            else if (i > 0 && m_bitMask.IsValid(k - width))
            {
              delta -= data[m - width * nDim];
            }
            else
              delta -= prevVal;

            prevVal = val;

            // bit stuff the huffman code for this delta
            int kBin = offset + (int)delta;
            int len = m_huffmanCodes[kBin].first;
            if (len <= 0)
              return false;

            unsigned int code = m_huffmanCodes[kBin].second;

            if (32 - bitPos >= len)
            {
              if (bitPos == 0)
                *dstPtr = 0;

              *dstPtr |= code << (32 - bitPos - len);
              bitPos += len;
              if (bitPos == 32)
              {
                bitPos = 0;
                dstPtr++;
              }
            }
            else
            {
              bitPos += len - 32;
              *dstPtr++ |= code >> bitPos;
              *dstPtr = code << (32 - bitPos);
            }
          }
    }
  }

  else if (m_imageEncodeMode == IEM_Huffman)
  {
    for (int k = 0, m0 = 0, i = 0; i < height; i++)
      for (int j = 0; j < width; j++, k++, m0 += nDim)
        if (m_bitMask.IsValid(k))
          for (int m = 0; m < nDim; m++)
          {
            T val = data[m0 + m];

            // bit stuff the huffman code for this val
            int kBin = offset + (int)val;
            int len = m_huffmanCodes[kBin].first;
            if (len <= 0)
              return false;

            unsigned int code = m_huffmanCodes[kBin].second;

            if (32 - bitPos >= len)
            {
              if (bitPos == 0)
                *dstPtr = 0;

              *dstPtr |= code << (32 - bitPos - len);
              bitPos += len;
              if (bitPos == 32)
              {
                bitPos = 0;
                dstPtr++;
              }
            }
            else
            {
              bitPos += len - 32;
              *dstPtr++ |= code >> bitPos;
              *dstPtr = code << (32 - bitPos);
            }
          }
  }

  else
    return false;

  size_t numUInts = dstPtr - arr + (bitPos > 0 ? 1 : 0) + 1;    // add one more as the decode LUT can read ahead
  *ppByte += numUInts * sizeof(unsigned int);
  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::DecodeHuffman(const Byte** ppByte, size_t& nBytesRemainingInOut, T* data) const
{
  if (!data || !ppByte || !(*ppByte))
    return false;

  Huffman huffman;
  if (!huffman.ReadCodeTable(ppByte, nBytesRemainingInOut, m_headerInfo.version))    // header and code table
    return false;

  int numBitsLUT = 0;
  if (!huffman.BuildTreeFromCodes(numBitsLUT))
    return false;

  int offset = (m_headerInfo.dt == DT_Char) ? 128 : 0;
  int height = m_headerInfo.nRows;
  int width = m_headerInfo.nCols;
  int nDim = m_headerInfo.nDim;

  const unsigned int* arr = (const unsigned int*)(*ppByte);
  const unsigned int* srcPtr = arr;
  int bitPos = 0;
  size_t nBytesRemaining = nBytesRemainingInOut;

  if (m_headerInfo.numValidPixel == width * height)    // all valid
  {
    if (m_imageEncodeMode == IEM_DeltaHuffman)
    {
      for (int iDim = 0; iDim < nDim; iDim++)
      {
        T prevVal = 0;
        for (int m = iDim, i = 0; i < height; i++)
          for (int j = 0; j < width; j++, m += nDim)
          {
            int val = 0;
            if (nBytesRemaining >= 4 * sizeof(unsigned int))
            {
              if (!huffman.DecodeOneValue_NoOverrunCheck(&srcPtr, nBytesRemaining, bitPos, numBitsLUT, val))
                return false;
            }
            else
            {
              if (!huffman.DecodeOneValue(&srcPtr, nBytesRemaining, bitPos, numBitsLUT, val))
                return false;
            }

            T delta = (T)(val - offset);

            if (j > 0)
              delta += prevVal;    // use overflow
            else if (i > 0)
              delta += data[m - width * nDim];
            else
              delta += prevVal;

            data[m] = delta;
            prevVal = delta;
          }
      }
    }

    else if (m_imageEncodeMode == IEM_Huffman)
    {
      for (int k = 0, m0 = 0, i = 0; i < height; i++)
        for (int j = 0; j < width; j++, k++, m0 += nDim)
          for (int m = 0; m < nDim; m++)
          {
            int val = 0;
            if (nBytesRemaining >= 4 * sizeof(unsigned int))
            {
              if (!huffman.DecodeOneValue_NoOverrunCheck(&srcPtr, nBytesRemaining, bitPos, numBitsLUT, val))
                return false;
            }
            else
            {
              if (!huffman.DecodeOneValue(&srcPtr, nBytesRemaining, bitPos, numBitsLUT, val))
                return false;
            }

            data[m0 + m] = (T)(val - offset);
          }
    }

    else
      return false;
  }

  else    // not all valid
  {
    if (m_imageEncodeMode == IEM_DeltaHuffman)
    {
      for (int iDim = 0; iDim < nDim; iDim++)
      {
        T prevVal = 0;
        for (int k = 0, m = iDim, i = 0; i < height; i++)
          for (int j = 0; j < width; j++, k++, m += nDim)
            if (m_bitMask.IsValid(k))
            {
              int val = 0;
              if (nBytesRemaining >= 4 * sizeof(unsigned int))
              {
                if (!huffman.DecodeOneValue_NoOverrunCheck(&srcPtr, nBytesRemaining, bitPos, numBitsLUT, val))
                  return false;
              }
              else
              {
                if (!huffman.DecodeOneValue(&srcPtr, nBytesRemaining, bitPos, numBitsLUT, val))
                  return false;
              }

              T delta = (T)(val - offset);

              if (j > 0 && m_bitMask.IsValid(k - 1))
              {
                delta += prevVal;    // use overflow
              }
              else if (i > 0 && m_bitMask.IsValid(k - width))
              {
                delta += data[m - width * nDim];
              }
              else
                delta += prevVal;

              data[m] = delta;
              prevVal = delta;
            }
      }
    }

    else if (m_imageEncodeMode == IEM_Huffman)
    {
      for (int k = 0, m0 = 0, i = 0; i < height; i++)
        for (int j = 0; j < width; j++, k++, m0 += nDim)
          if (m_bitMask.IsValid(k))
            for (int m = 0; m < nDim; m++)
            {
              int val = 0;
              if (nBytesRemaining >= 4 * sizeof(unsigned int))
              {
                if (!huffman.DecodeOneValue_NoOverrunCheck(&srcPtr, nBytesRemaining, bitPos, numBitsLUT, val))
                  return false;
              }
              else
              {
                if (!huffman.DecodeOneValue(&srcPtr, nBytesRemaining, bitPos, numBitsLUT, val))
                  return false;
              }

              data[m0 + m] = (T)(val - offset);
            }
    }

    else
      return false;
  }

  size_t numUInts = srcPtr - arr + (bitPos > 0 ? 1 : 0) + 1;    // add one more as the decode LUT can read ahead
  size_t len = numUInts * sizeof(unsigned int);

  if (nBytesRemainingInOut < len)
    return false;

  *ppByte += len;
  nBytesRemainingInOut -= len;
  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::WriteMinMaxRanges(const T* /*data*/, Byte** ppByte) const
{
  if (!ppByte || !(*ppByte))
    return false;

  //printf("write min / max = %f  %f\n", m_zMinVec[0], m_zMaxVec[0]);

  int nDim = m_headerInfo.nDim;
  if (/* nDim < 2 || */ (int)m_zMinVec.size() != nDim || (int)m_zMaxVec.size() != nDim)
    return false;

  std::vector<T> zVec(nDim);
  size_t len = nDim * sizeof(T);

  for (int i = 0; i < nDim; i++)
    zVec[i] = (T)m_zMinVec[i];

  memcpy(*ppByte, &zVec[0], len);
  (*ppByte) += len;

  for (int i = 0; i < nDim; i++)
    zVec[i] = (T)m_zMaxVec[i];

  memcpy(*ppByte, &zVec[0], len);
  (*ppByte) += len;

  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::ReadMinMaxRanges(const Byte** ppByte, size_t& nBytesRemaining, const T* /*data*/)
{
  if (!ppByte || !(*ppByte))
    return false;

  int nDim = m_headerInfo.nDim;

  m_zMinVec.resize(nDim);
  m_zMaxVec.resize(nDim);

  std::vector<T> zVec(nDim);
  size_t len = nDim * sizeof(T);

  if (nBytesRemaining < len || !memcpy(&zVec[0], *ppByte, len))
    return false;

  (*ppByte) += len;
  nBytesRemaining -= len;

  for (int i = 0; i < nDim; i++)
    m_zMinVec[i] = zVec[i];

  if (nBytesRemaining < len || !memcpy(&zVec[0], *ppByte, len))
    return false;

  (*ppByte) += len;
  nBytesRemaining -= len;

  for (int i = 0; i < nDim; i++)
    m_zMaxVec[i] = zVec[i];

  //printf("read min / max = %f  %f\n", m_zMinVec[0], m_zMaxVec[0]);

  return true;
}

// -------------------------------------------------------------------------- ;

template<class T>
bool Lerc2::FillConstImage(T* data) const
{
  if (!data)
    return false;

  const HeaderInfo& hd = m_headerInfo;
  int nCols = hd.nCols;
  int nRows = hd.nRows;
  int nDim = hd.nDim;
  T z0 = (T)hd.zMin;

  if (nDim == 1)
  {
    for (int k = 0, i = 0; i < nRows; i++)
      for (int j = 0; j < nCols; j++, k++)
        if (m_bitMask.IsValid(k))
          data[k] = z0;
  }
  else
  {
    std::vector<T> zBufVec(nDim, z0);

    if (hd.zMin != hd.zMax)
    {
      if ((int)m_zMinVec.size() != nDim)
        return false;

      for (int m = 0; m < nDim; m++)
        zBufVec[m] = (T)m_zMinVec[m];
    }

    int len = nDim * sizeof(T);
    for (int k = 0, m = 0, i = 0; i < nRows; i++)
      for (int j = 0; j < nCols; j++, k++, m += nDim)
        if (m_bitMask.IsValid(k))
          memcpy(&data[m], &zBufVec[0], len);
  }

  return true;
}

// -------------------------------------------------------------------------- ;

