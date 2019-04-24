# mini-archiver

MiniArchiver is C++17 compatible header only, light, archiver library that can handle loading/saving of 

   - all basic types,
   - classes
   - several STL containers
   - most of [glm](https://glm.g-truc.net) types
   
 Library is modeled by the idea made by cereal, but with C++17/C++14 compatibility in mind. If you are familiar with the cereal, it will be 
 easy to switch to mini-archiver.
 
  ### Guideline for loading/saving:
  
  These are the steps to save data into a bnary file.
 
 ```
    // open a file   
    BinaryOut file("file.bin", flag);
```
  
  where the flag can be one of the following: 

 * `STREAM_TYPE_UNCOMPRESSED` (default type) when stream is saved uncompressed,
 * `STREAM_TYPE_COMPRESSED` when the stream is saved with zlib compression
 * or `STREAM_TYPE_COMPRESSED | STREAM_TYPE_ENCODED`, when stream is saved zlib compressed and encoded with simple encoding system
  
```
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
```

  Loading from binary file, either compressed or not, is made as follows:
  
    BinaryIn file("file.bin");

    if( file.get_stream_type() & STREAM_TYPE_ENCODED )
	    file.set_encode_table({ 0x1, 0x2, 0x3, 0x4}); // can be of any length

    if (file.get_stream_type() & STREAM_TYPE_COMPRESSED)
	    file.read_compressed_data();

    // setup some variables...
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

    // and load the data
    file(c, foo, intvec, test_str, fooptr, intptr, foovec, mymap, mypair);

    file.close();
    
 Serialization of class data is made simple by adding Serialize method to a class like in example below:

```
    class Foo {
	  public:
	  int value_ = 11;
	  template <class Archive>
	  void Serialize( Archive& ar, unsigned version) {
	  	ar(value_);
	  }
    };
```
Saving the data:
```
    Foo foo;
    BinaryOut file("file.bin");
    file(foo);
    file.close();
```
And loading the data:
 ```
    Foo foo;
    BinaryIn file("file.bin");
    file(foo);
    file.close();
```   

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
