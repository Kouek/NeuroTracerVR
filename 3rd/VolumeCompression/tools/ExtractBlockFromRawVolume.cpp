//
// Created by wyz on 2022/2/21.
//
#include <cmdline.hpp>
#include <fstream>
#include <iostream>
#include <VoxelCompression/voxel_compress/VoxelCmpDS.h>
int main(int argc,char** argv){
    cmdline::parser cmd;

    cmd.add<std::string>("input",0,"input filename",true);
    cmd.add<std::string>("output",0,"output filename",true);
    cmd.add<uint32_t>("raw_x",0,"raw volume width",true);
    cmd.add<uint32_t>("raw_y",0,"raw volume height",true);
    cmd.add<uint32_t>("raw_z",0,"raw volume depth",true);
    cmd.add<uint32_t>("sub_x",0,"block start x pos",true);
    cmd.add<uint32_t>("sub_y",0,"block start y pos",true);
    cmd.add<uint32_t>("sub_z",0,"block start z pos",true);
    cmd.add<uint32_t>("sub_width",0,"block width",true);
    cmd.add<uint32_t>("sub_height",0,"block height",true);
    cmd.add<uint32_t>("sub_depth",0,"block depth",true);

    cmd.parse_check(argc,argv);

    uint32_t raw_x = cmd.get<uint32_t>("raw_x");
    uint32_t raw_y = cmd.get<uint32_t>("raw_y");
    uint32_t raw_z = cmd.get<uint32_t>("raw_z");
    uint32_t x = cmd.get<uint32_t>("sub_x");
    uint32_t y = cmd.get<uint32_t>("sub_y");
    uint32_t z = cmd.get<uint32_t>("sub_z");
    uint32_t width = cmd.get<uint32_t>("sub_width");
    uint32_t height = cmd.get<uint32_t>("sub_height");
    uint32_t depth = cmd.get<uint32_t>("sub_depth");

    std::string input = cmd.get<std::string>("input");
    std::string output = cmd.get<std::string>("output");

    try{

        std::ifstream in(input, std::ios::binary);
        std::ofstream out(output,std::ios::binary);
        if (!in.is_open() || !out.is_open())
        {
            throw std::runtime_error("file open failed");
        }
        //todo x y z out-of range
        std::vector<uint8_t> buffer(width);
        for(uint32_t kz = 0; kz < depth; kz++){
            for(uint32_t ky = 0; ky < height; ky++){
                size_t pos = (size_t)(kz + z) * raw_x * raw_y + (size_t)(ky + y) * raw_x + x;
                in.seekg(pos,std::ios::beg);
                in.read(reinterpret_cast<char*>(buffer.data()),width);
                pos = (size_t)kz * width * height + ky * width;
                out.seekp(pos,std::ios::beg);
                out.write(reinterpret_cast<char*>(buffer.data()),width);
            }
        }
        in.close();
        out.close();
    }
    catch (const std::exception& err)
    {
        std::cerr<<err.what()<<std::endl;
    }

    return 0;
}