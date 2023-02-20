//
// Created by wyz on 2021/5/20.
//
#define TinyTIFFIMPL
#include <VoxelCompression/utils/util.h>
#include <cmdline.hpp>
#include <iomanip>
#include <sstream>
#include <fstream>

int main(int argc,char** argv)
{
    cmdline::parser cmd;


    cmd.add<std::string>("in",'i',"input file path",true);
    cmd.add<std::string>("prefix",'p',"prefix for input is tif format",false);
    cmd.add<std::string>("out",'o',"output file name",true);
    cmd.add<std::string>("out_dir",'d',"output file directory",false,".");
    cmd.add<int>("raw_x",'x',"x resolution of raw volume data",true);
    cmd.add<int>("raw_y",'y',"y resolution of raw volume data",true);
    cmd.add<int>("raw_z",'z',"z resolution of raw volume data",true);

    cmd.parse_check(argc,argv);

    auto input_path=cmd.get<std::string>("in");
    auto prefix=cmd.get<std::string>("prefix");
    auto output_name=cmd.get<std::string>("out");
    auto output_dir=cmd.get<std::string>("out_dir");
    auto output_path=output_dir+"/"+output_name;
    auto raw_x=cmd.get<int>("raw_x");
    auto raw_y=cmd.get<int>("raw_y");
    auto raw_z=cmd.get<int>("raw_z");

    std::vector<uint8_t> slice;
    size_t slice_size=(size_t)raw_x*raw_y;
    slice.resize(slice_size);

    std::ofstream out(output_path,std::ios::binary);
    for(int i=0;i<raw_z;i++){
        std::stringstream ss;
        ss<<input_path<<"/"<<prefix<<std::setw(5)<<std::setfill('0')<<std::to_string(i+1)<<".tif";
        std::cout<<"loading "<<ss.str()<<std::endl;
        load_volume_tif(slice,ss.str());
        out.write(reinterpret_cast<char*>(slice.data()),slice.size());
    }
    out.close();

    return 0;
}
