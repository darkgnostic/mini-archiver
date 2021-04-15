#pragma once

/* MiniArchiver.h -- interface of the 'miniarchiver' serialization library
  version 1.0, April 24th, 2019

  Copyright (C) 2019- Hodi Isidor aka darkgnostic

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
	 claim that you wrote the original software. If you use this software
	 in a product, an acknowledgment in the product documentation would be
	 appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
	 misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
  
*/

/*
  MiniArchiver is C+17 compatible header only archiver library that can handle loading/saving of 

   - all basic types,
   - classes
   - several STL containers
   - basic glm types
   
  Serialization of class data is made simple by adding Serialize method to a class as in example below:

  class Foo {
	public:
	int value_ = 11;
	template <class Archive>
	void Serialize( Archive& ar, unsigned version) {
		ar(value_);
	}
  };

  Foo foo;
  BinaryOut file("file.bin");
  file(foo);

  Serialization of base classes is not made automatically, you need to take care of handling it.

  class FooDerived : public Foo  {
  public:
	int derived_ = 12;
	template <class Archive>
	void Serialize(Archive& ar, unsigned version) {
		Foo::Serialize(ar, version);
		ar(derived_);
	}
  };

  FooDerived foo;
  BinaryOut file("file.bin");
  file(foo);

  Guideline for loading/saving:
  ##################################################
  Saving into binary file, one uncompressed stream of data.

  BinaryOut file("file.bin", flag);
  ...
  flag can be one of the following: 

    * STREAM_TYPE_UNCOMPRESSED (default type) when stream is saved uncompressed,
	* STREAM_TYPE_COMPRESSED when stream is saved with zlib compression
	* or STREAM_TYPE_COMPRESSED | STREAM_TYPE_ENCODED, when stream is saved zlib compressed and encoded with simple encoding system
  ...

  // save actual data...
  FooDerived foo;
  int c = 42;
  std::string test_str = "teststring";
  std::vector<int> intvec{ 1,2,3,4,5,6,7,8,9 };
  std::shared_ptr<FooDerived> fooptr = std::make_shared<FooDerived>(FooDerived());
  std::shared_ptr<int> intptr = std::make_shared<int>(5);
  std::vector< std::shared_ptr<FooDerived> > foovec(5, std::make_shared<FooDerived>());
  std::map<int, std::string> mymap;
  std::pair<std::string, int> mypair = std::make_pair("pair1", 2);
  mymap[1] = "mapval";

  file(c, foo, intvec, test_str, fooptr, intptr, foovec, mymap, mypair);

  if( file.get_stream_type() & STREAM_TYPE_ENCODED )
	file.set_encode_table({ 0x1, 0x2, 0x3, 0x4}); // can be of any length

  if( file.get_stream_type() & STREAM_TYPE_COMPRESSED )
	file.compress_and_save();

  file.close();

  ##################################################
  Loading from binary file, either compressed or not, is made as follows:
  
  BinaryIn file("file.bin");

  if( file.get_stream_type() & STREAM_TYPE_ENCODED )
	file.set_encode_table({ 0x1, 0x2, 0x3, 0x4}); // can be of any length

  if (file.get_stream_type() & STREAM_TYPE_COMPRESSED)
	file.read_compressed_data();

  // load actual data...
  FooDerived foo;
  int c = 42;
  std::string test_str = "teststring";
  std::vector<int> intvec{ 1,2,3,4,5,6,7,8,9 };
  std::shared_ptr<FooDerived> fooptr = std::make_shared<FooDerived>(FooDerived());
  std::shared_ptr<int> intptr = std::make_shared<int>(5);
  std::vector< std::shared_ptr<FooDerived> > foovec(5, std::make_shared<FooDerived>());
  std::map<int, std::string> mymap;
  std::pair<std::string, int> mypair = std::make_pair("pair1", 2);
  mymap[1] = "mapval";

  file(c, foo, intvec, test_str, fooptr, intptr, foovec, mymap, mypair);

  file.close();
*/

#define MINIARCHIVER_VERSION 10

// defines for a binary file data types
#define STREAM_TYPE_UNCOMPRESSED		0
#define STREAM_TYPE_COMPRESSED			1
#define STREAM_TYPE_ENCODED				2

// optionals:
// enable load/save of std::string, std::array, std::bitset, std::pair, std::vector, std::map, std::shared_ptr
#define MINIARCHIVER_USE_STL	1
#define MINI_ARCHIVER_USE_ZLIB	1
//#define _DEBUG_OUT

#include <iostream>
#include <fstream>
#include <type_traits>
#include <typeinfo>
#include <string>
#ifdef __GNUC__
#ifdef __MINGW32__
#include <cstring>
#include <memory>
#endif
#endif

#if defined MINIARCHIVER_USE_STL
#include <vector>
#include <map>
#include <array>
#include <bitset>
#endif

#if defined MINI_ARCHIVER_USE_ZLIB
#include <sstream>
#include <zlib.h>
#endif

#pragma warning(disable : 5033)

enum class LoadState {
    LoadOk,
    SaveOk,
    CantOpenFailed,
    FileEncoded,
    WrongKey
};


using binary_vector = std::vector<unsigned char>;

template<typename T, typename = void>
struct has_serialize : std::false_type { };

template<typename T>
struct has_serialize<T, decltype(std::declval<T&>().Serialize(std::declval<int&>(), unsigned()), void())> : std::true_type { };

template<typename DataType, typename ArchiveType>
typename std::enable_if<has_serialize<DataType>::value, void>::type
CallSerialize(DataType& t, ArchiveType& ar, unsigned version) {
#if defined _DEBUG_OUT
	std::cout << "Archiver '" << typeid(ar).name() << "'" << std::endl;
#endif
	t.Serialize(ar, version);
}

template<typename DataType, typename ArchiveType>
typename std::enable_if<!has_serialize<DataType>::value, void>::type
CallSerialize(DataType& t, ArchiveType& ar, unsigned version) {
	static_assert(std::is_fundamental<DataType>::value, "Object invoked for serialization, but Serialization method doesn't exist.");
}

/*!
 * \namespace mini_archiver
 *
 * \brief: future JSON read/write implementation prerequisite
 *
 */
namespace mini_archiver {
	template <typename DataType>
	DataType& make_nvp(const std::string& name, DataType& value) {
#if defined _DEBUG_OUT
	    std:: cout << name.c_str() << ": " << sizeof(value) << std::endl;
#endif
		return value;
	}
	template <typename DataType>
	DataType& make_nvp(const std::string& name, DataType&& value) {
		return value;
	}
}

/*!
 * \class ArchiveBase
 *
 * \brief
 *
 */
class ArchiveBase {
protected:
	int version_ = MINIARCHIVER_VERSION;
public:
    ArchiveBase() = default;
    virtual ~ArchiveBase() = default;

	int Version() {
		return version_;
	}
	virtual bool IsLoader() = 0;
protected:
    unsigned short hash_ = 0;
    binary_vector encode_table_;

	template <typename DataType>
	void archive_raw(DataType& data) {
		archive((char*)&data, sizeof(DataType));
	}

	virtual void archive(char* ptr, unsigned size) = 0;

#if defined MINI_ARCHIVER_USE_ZLIB
    template<typename T>
    unsigned short crc16(const unsigned char* data_p, T length){
        unsigned char x;
        unsigned short crc = 0xFFFF;

        while (length--){
            x = crc >> 8 ^ *data_p++;
            x ^= x>>4;
            crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
        }
        return crc;
    }

    void set_encode_table(const binary_vector& encode_table) {
        encode_table_ = encode_table;
        hash_ = crc16(encode_table_.data(), encode_table_.size());
        // 0 is return value for bad hash, inc value
        if( !hash_ ) hash_++;
    }
#endif
};




/*!
 * \class ArchiveOut
 *
 * \brief
 *
 */
class ArchiveOut : public ArchiveBase
{
public:
	ArchiveOut() = default;
	virtual ~ArchiveOut() = default;
	
	template <typename DataType>
	auto operator()(DataType& data) {

		if (has_serialize<DataType>::value)
			CallSerialize(data, *this, Version());
		else {
#if defined _DEBUG_OUT
		   //auto name = std::string(typeid(data).name());
			//std::cout << "Saving fundamental '" << name.c_str() << "' (" << sizeof(data) << ")" << std::endl;
#endif
			archive_raw<DataType>(data);
		}

		return this;
	}

	template <typename DataType, typename ...Args>
	auto operator()(DataType& data, Args&... args) {
		operator()(data);
		return operator()(args...);
	}

#if defined MINIARCHIVER_USE_STL
    auto operator()(std::string& data) {
#if defined _DEBUG_OUT
        std::cout << "Saving '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
        unsigned size = static_cast<unsigned>(data.size());
        archive_raw<unsigned>(size);
        for (auto& elem : data) {
            operator()(elem);
        }
        return this;
    }

	// special STL cases
	template <typename std::size_t Size>
	auto operator()(std::bitset<Size>& data) {
#if defined _DEBUG_OUT
		std::cout << "Saving bitset '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
		std::string d = data.to_string();
		operator()(d);
		return this;
	}
	
	template <typename DataType>
	auto operator()(std::vector<DataType>& data) {
#if defined _DEBUG_OUT
		std::cout << "Saving vector '" << "' (" << sizeof(data) << ")" << std::endl;
#endif
		unsigned size = (unsigned)data.size();
		archive_raw<unsigned>(size);

		for (auto& elem : data) {
			operator()(elem);
		}

		return this;
	}
	
	// special STL cases
	template <typename DataType, std::size_t Size>
	auto operator()(std::array<DataType, Size>& data) {
#if defined _DEBUG_OUT
		std::cout << "Saving array '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
		unsigned size = (unsigned)data.size();
		archive_raw<unsigned>(size);

		for (auto& elem : data) {
			operator()(elem);
		}

		return this;
	}

	template <typename DataType>
	auto operator()(std::shared_ptr<DataType>& data) {
#if defined _DEBUG_OUT
		std::cout << "Saving '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
		operator()(*data);
		return this;
	}

	template <typename DataType, typename DataType2>
	auto operator()(std::map<DataType, DataType2>& data) {
#if defined _DEBUG_OUT
		std::cout << "Saving '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
		unsigned size = static_cast<unsigned>(data.size());
		archive_raw<unsigned>(size);
		for (auto& elem : data) {
			operator()(elem);	// will call std::pair
		}
		return this;
	}

	template <typename DataType, typename DataType2>
	auto operator()(std::pair<DataType, DataType2>& data) {
#if defined _DEBUG_OUT
		std::cout << "Saving '" << typeid(data).name() << "'" << std::endl;
#endif
		operator()(data.first, data.second);
		return this;
	}
#endif

#if defined glm_glm
	// GLM loader/saver
	template < typename DataType >
	auto operator()(glm::detail::tvec2<DataType>& data) {
		operator()(data.x, data.y);
		return this;
	}
	template < typename DataType >
	auto operator()(glm::detail::tvec3<DataType>& data) {
		operator()(data.x, data.y, data.z);
		return this;
	}
	template < typename DataType >
	auto operator()(glm::detail::tvec4<DataType>& data) {
		operator()(data.x, data.y, data.z, data.w);
		return this;
	}
#else
    // GLM loader/saver
	template < typename DataType >
	auto operator()(glm::tvec2<DataType>& data) {
		operator()(data.x, data.y);
		return this;
	}
	template < typename DataType >
	auto operator()(glm::tvec3<DataType>& data) {
		operator()(data.x, data.y, data.z);
		return this;
	}
	template < typename DataType >
	auto operator()(glm::tvec4<DataType>& data) {
		operator()(data.x, data.y, data.z, data.w);
		return this;
	}
#endif
	virtual bool IsLoader() override {
		return false;
	}
};

/*!
 * \class ArchiveIn
 *
 * \brief
 *
 */
class ArchiveIn : public ArchiveBase
{
public:
	ArchiveIn() = default;
	virtual ~ArchiveIn() = default;

	template <typename DataType>
	auto operator()(DataType& data) {

		if (has_serialize<DataType>::value)
			CallSerialize(data, *this, Version());
		else {
#if defined _DEBUG_OUT
			//std::cout << "Loading fundamental '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
			archive_raw<DataType>(data);
		}

		return this;
	}

	template <typename DataType, typename ...Args>
	auto operator()(DataType& data, Args&... args) {
		operator()(data);
		return operator()(args...);
	}

#if defined MINIARCHIVER_USE_STL
    auto operator()(std::string& data) {
#if defined _DEBUG_OUT
        std::cout << "Loading '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
        unsigned size = 0;
        archive_raw<unsigned>(size);
        data.resize(size);
        for (auto& elem : data) {
            operator()(elem);
        }
        return this;
    }
	// special STL cases
	template <typename std::size_t Size>
	auto operator()(std::bitset<Size>& data) {
#if defined _DEBUG_OUT
		std::cout << "Loading bitset '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
		std::string string_bitset;
		operator()(string_bitset);
		
		data = std::bitset<Size>(string_bitset);
		return this;
	}
	
	template <typename DataType>
	auto operator()(std::vector<DataType>& data) {
#if defined _DEBUG_OUT
		std::cout << "Loading vector '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
		unsigned size = 0;
		archive_raw<unsigned>(size);
		data.resize(size);
		for (auto& elem : data) {
			operator()(elem);
		}

		return this;
	}

	// special STL cases
	template <typename DataType, std::size_t Size>
	auto operator()(std::array<DataType, Size>& data) {
#if defined _DEBUG_OUT
		std::cout << "Loading array '" << typeid(data).name() << "' (" << Size << ")" << std::endl;
#endif
		unsigned size = 0;
		archive_raw<unsigned>(size);
		for (auto& elem : data) {
			operator()(elem);
		}

		return this;
	}

	template <typename DataType>
	auto operator()(std::shared_ptr<DataType>& data) {
#if defined _DEBUG_OUT
		std::cout << "Loading '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
		if (data == nullptr)
			data = std::make_shared<DataType>();
		operator()(*data);
		return this;
	}


	template <typename DataType, typename DataType2>
	auto operator()(std::map<DataType, DataType2>& data) {
#if defined _DEBUG_OUT
		std::cout << "Loading '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
		unsigned size = 0;
		archive_raw<unsigned>(size);
		for (unsigned i = 0; i < size; i++) {
			DataType e_first;	operator()(e_first);
			DataType2 e_second;	operator()(e_second);
			data[e_first] = e_second;
		}
		return this;
	}

	template <typename DataType, typename DataType2>
	auto operator()(std::pair<DataType, DataType2>& data) {
#if defined _DEBUG_OUT
		std::cout << "Loading '" << typeid(data).name() << "' (" << sizeof(data) << ")" << std::endl;
#endif
		operator()(data.first, data.second);
		return this;
	}
#endif

#if defined glm_glm
	// GLM loader/saver
	template < typename DataType >
	auto operator()(glm::detail::tvec2<DataType>& data) {
		operator()(data.x, data.y);
		return this;
	}
	template < typename DataType >
	auto operator()(glm::detail::tvec3<DataType>& data) {
		operator()(data.x, data.y, data.z);
		return this;
	}
	template < typename DataType >
	auto operator()(glm::detail::tvec4<DataType>& data) {
		operator()(data.x, data.y, data.z, data.w);
		return this;
	}
#else
// GLM loader/saver
    template < typename DataType >
    auto operator()(glm::tvec2<DataType>& data) {
        operator()(data.x, data.y);
        return this;
    }
    template < typename DataType >
    auto operator()(glm::tvec3<DataType>& data) {
        operator()(data.x, data.y, data.z);
        return this;
    }
    template < typename DataType >
    auto operator()(glm::tvec4<DataType>& data) {
        operator()(data.x, data.y, data.z, data.w);
        return this;
    }
#endif

	virtual bool IsLoader() override {
		return true;
	}
};


/*!
 * \class OutputMemoryStream
 *
 * \brief: writes a stream of binary data to a memory
 *
 */
class OutputMemoryStream : public ArchiveOut, public std::vector<unsigned char> {
public:
	OutputMemoryStream() = default;
	virtual ~OutputMemoryStream() = default;

	template <class Archive>
	void Serialize(Archive& ar, unsigned version) {
#if defined _DEBUG_OUT
		std::cout << "Archiving '" << typeid(*this).name() << "'" << std::endl;
#endif
		std::vector<unsigned char>* ptr = this;
		ar(*ptr);
	}

protected:
	friend class BinaryOut;
	virtual void archive(char* ptr, unsigned size) override
	{
		for (unsigned i = 0; i < size; i++)
			push_back(ptr[i]);
	}
};

/*!
 * \class InputMemoryStream
 *
 * \brief: reads a stream of binary data from memory
 *
 */
class InputMemoryStream : public ArchiveIn, public std::vector<unsigned char> {
	unsigned read_offset_ = 0;
public:
	InputMemoryStream() = default;
	InputMemoryStream(const std::vector<unsigned char>& vec) {
		for (auto& v : vec)
			push_back(v);
	}

	virtual ~InputMemoryStream() = default;
	
	template <class Archive>
	void Serialize(Archive& ar, unsigned version) {
#if defined _DEBUG_OUT
		std::cout << "Archiving '" << typeid(*this).name() << "'" << std::endl;
#endif
		std::vector<unsigned char>* ptr = this;
		ar(*ptr);
	}
//protected:
	friend class BinaryIn;
	virtual void archive(char* ptr, unsigned size) override	{
		std::memcpy(ptr, &this->operator[](read_offset_), size);
		read_offset_ += size;
	}
};

/*!
 * \class BinaryOut
 *
 * \brief: class responsible of saving binary data to a file
 *
 */
class BinaryOut : public ArchiveOut, public std::ofstream {

	const char* file_type_ = "AMF";
	short stream_type_ = STREAM_TYPE_UNCOMPRESSED;
#if defined MINI_ARCHIVER_USE_ZLIB
	OutputMemoryStream	data_stream_;	// used if compressed/encoded
#endif
public:
	BinaryOut(const char* fname, const binary_vector& encode_table, short stream_type = STREAM_TYPE_UNCOMPRESSED ) :
		std::ofstream(fname, std::ios::binary) {

		archive((char*)mini_archiver::make_nvp("MagicNumber",file_type_), 4);
		archive_raw<int>(mini_archiver::make_nvp("Version", version_));

		if (version_ >= 6) {
#ifndef MINI_ARCHIVER_USE_ZLIB
			// will be saved raw if there is no zlib header included
			stream_type = STREAM_TYPE_UNCOMPRESSED;
#endif
			archive_raw<short>(mini_archiver::make_nvp("StreamType", stream_type));
		}

		stream_type_ = stream_type;

        if (stream_type_ & STREAM_TYPE_ENCODED)
           set_encode_table(encode_table);

        if( !hash_ ) hash_ = 1;

        if( version_ >= 10 ) {
            write_uncompressed(mini_archiver::make_nvp("Hash", hash_));
        }
	}
	~BinaryOut() {
		//flush();

	}

    template<class DataType>
	LoadState save(DataType& data ) {
	    // this will either write to disk or to memory buffer if compressed
	    operator ()(data);
	    return LoadState::SaveOk;
	}

	void finalize() {
        if (get_stream_type() & STREAM_TYPE_COMPRESSED) {
            // here we will compress memory buffer (and eventuallye ncode it)
            compress_and_save();
        }
	}
private:

    short get_stream_type() const {
        return stream_type_;
    }

	// compress data and send compressed data to disk
	void compress_and_save() {

		if (is_open() && stream_type_ & STREAM_TYPE_COMPRESSED) {
			// we have memory data stored
#if defined MINI_ARCHIVER_USE_ZLIB
            // we are writing from now to disk
            auto stream_type_old = stream_type_;

            stream_type_ = STREAM_TYPE_UNCOMPRESSED;

            uLong stream_size = (uLong)data_stream_.size();
            uLong computed_size = compressBound(stream_size);
            binary_vector processed_data(computed_size);
            // Deflate
            int error = compress((Bytef *)processed_data.data(), &computed_size, data_stream_.data(), stream_size);
            // here if error code is Z_OK the variable compSize contains size of
            // compressed data that is usually smaller than initial buffer size.
            // vector size may be adjusted to actual size of compressed data
            if (error == Z_OK) {
                processed_data.resize(computed_size);

                if (stream_type_old & STREAM_TYPE_ENCODED) {
                    for (size_t i = 2; i < processed_data.size(); i++) {
                        processed_data[i] += encode_table_[(i + stream_size) % encode_table_.size()];
                    }
                }

                operator()(processed_data);
                archive_raw(stream_size);	// unpacked size
            }

            stream_type_ = stream_type_old;
#endif
		}
	}

protected:
    template <typename DataType>
    void write_uncompressed(DataType& data) {
        auto stream_old = stream_type_;
        stream_type_ = STREAM_TYPE_UNCOMPRESSED;
        archive_raw(data);
        stream_type_ = stream_old;
    }
	void archive(char* ptr, unsigned size) override {
		if (stream_type_ == STREAM_TYPE_UNCOMPRESSED)
			write(ptr, size);
#if defined MINI_ARCHIVER_USE_ZLIB
		else
			data_stream_.archive(ptr, size);
#endif
	}
};

/*!
 * \class BinaryIn
 *
 * \brief: class responsible of loading binary data from a file
 *
 */
class BinaryIn : public ArchiveIn, public std::ifstream {
	char file_type_[4];
	short stream_type_ = 0;
#if defined MINI_ARCHIVER_USE_ZLIB
	InputMemoryStream	data_stream_;
#endif
public:
	BinaryIn(const std::string& file_path, const binary_vector& encode_table) :
		std::ifstream(file_path, std::ios::binary)
	{
		archive((char*)mini_archiver::make_nvp("MagicNumber",file_type_), 4);
		archive_raw<int>(mini_archiver::make_nvp("Version",version_));

		if (version_ >= 6) {
			archive_raw<short>(mini_archiver::make_nvp("StreamType",stream_type_));
		}

		if( stream_type_ & STREAM_TYPE_ENCODED )
            set_encode_table(encode_table);
	}

    template <typename DataType>
	LoadState load(DataType& data) {

        auto ret = pre_load();
        if( ret == LoadState::LoadOk) {
            operator()(data);
            return LoadState::LoadOk;
        }

	    return ret;
	}

    LoadState pre_load() {

        if (encode_table_.empty() && (stream_type_ & STREAM_TYPE_ENCODED) ) {
            return LoadState::FileEncoded;
        }

        if( version_ >= 10 ) {
            read_uncompressed(mini_archiver::make_nvp("Hash",hash_));
        }

        if (stream_type_ & STREAM_TYPE_COMPRESSED) {
            if (read_compressed_data() == 0)
                return LoadState::WrongKey;
        }

        return LoadState::LoadOk;
    }
private:

	unsigned short read_compressed_data() {

		if (stream_type_ & STREAM_TYPE_COMPRESSED) {
			short saved_stream_type = stream_type_;
			stream_type_ = STREAM_TYPE_UNCOMPRESSED;

#if defined MINI_ARCHIVER_USE_ZLIB
			InputMemoryStream	mem;
			mem.Serialize(*this, Version());

			uLong original_size = 0;
			archive_raw(mini_archiver::make_nvp("OriginalSize",original_size));	// load unpacked size

			if (saved_stream_type & STREAM_TYPE_ENCODED) {
				for (size_t i = 2; i < mem.size(); i++) {
					mem[i] -= encode_table_[(i + original_size) % encode_table_.size()];
				}
			}

			binary_vector processed_data(original_size);

			int error = uncompress((Bytef *)processed_data.data(), &original_size, mem.data(), (uLong)mem.size());

			if (error == Z_OK) {
				data_stream_.swap(processed_data);
			}
#endif
			stream_type_ = saved_stream_type;
		}

		return hash_;
	}

	short get_stream_type() const {
		return stream_type_;
	}

protected:
    template <typename DataType>
    void read_uncompressed(DataType& data) {
        auto stream_old = stream_type_;
        stream_type_ = STREAM_TYPE_UNCOMPRESSED;
        archive_raw(data);
        stream_type_ = stream_old;
	}
	void archive(char* ptr, unsigned size ) override {
		if(stream_type_ == STREAM_TYPE_UNCOMPRESSED )
			read(ptr, size);
#if defined MINI_ARCHIVER_USE_ZLIB
		else
			data_stream_.archive(ptr, size);
#endif
	}
};
