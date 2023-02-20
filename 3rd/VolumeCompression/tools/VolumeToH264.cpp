#include <VoxelCompression/utils/VolumeReader.h>
#include<VoxelCompression/voxel_compress/VoxelCmpDS.h>
#include<VoxelCompression/voxel_compress/VoxelCompress.h>
#include <cmdline.hpp>

int main(int argc,char** argv){
    try{
        cmdline::parser cmd;
        cmd.add<std::string>("input",'i',"input volume file path",true);
        cmd.add<int>("raw_x",'x'," x resolution of input raw volume",true);
        cmd.add<int>("raw_y",'y'," y resolution of input raw volume",true);
        cmd.add<int>("raw_z",'z'," z resolution of input raw volume",true);
        cmd.add<int>("log",'l'," log block length",true);
        cmd.add<int>("padding",'p',"padding of a block",false,1);
        cmd.add<std::string>("output",'o',"output h264 file path",true);
        cmd.add<std::string>("voxel_type",'t',"voxel type",false,"uint8",
                             cmdline::oneof<std::string>("uint8","int8","int16","uint16","float16",
                                            "uint32","int32","float32"));
        cmd.add<uint32_t>("encode_frame_w",0,"",true,0);
        cmd.add<uint32_t>("encode_frame_h",0,"",true,0);
        cmd.parse_check(argc,argv);

        auto input = cmd.get<std::string>("input");
        auto output = cmd.get<std::string>("output");
        uint32_t raw_x = cmd.get<int>("raw_x");
        uint32_t raw_y = cmd.get<int>("raw_y");
        uint32_t raw_z = cmd.get<int>("raw_z");
        uint32_t log = cmd.get<int>("log");
        uint32_t padding = cmd.get<int>("padding");
        uint32_t block_length = std::pow(2,log);
        uint32_t frame_width = cmd.get<uint32_t>("encode_frame_w");
        uint32_t frame_height = cmd.get<uint32_t>("encode_frame_h");
        uint32_t voxel_type = VOXEL_UNKNOWN;
        auto vt = cmd.get<std::string>("voxel_type");
        if(vt == "uint8"){
            voxel_type = VOXEL_UINT8;
        }
        else if(vt == "int8"){
            voxel_type = VOXEL_INT8;
        }
        else if(vt == "uint16"){
            voxel_type = VOXEL_UINT16;
        }
        else if(vt == "int16"){
            voxel_type = VOXEL_INT16;
        }
        else if(vt == "float16"){
            voxel_type = VOXEL_FLOAT16;
        }
        else if(vt =="uint32"){
            voxel_type = VOXEL_UINT32;
        }
        else if(vt == "int32"){
            voxel_type = VOXEL_INT32;
        }
        else if(vt == "float32"){
            voxel_type = VOXEL_FLOAT32;
        }
        BlockVolumeReader reader(
            {input,raw_x,raw_y,raw_z,sv::GetVoxelSize(voxel_type),block_length,padding}
            );
        auto dim = reader.getDim();

        sv::Header header{};
        header.identifier = VOXEL_COMPRESS_FILE_IDENTIFIER;
        header.version = VOXEL_COMPRESS_VERSION;
        header.raw_x = raw_x;
        header.raw_y = raw_y;
        header.raw_z = raw_z;
        header.block_dim_x = dim[0];
        header.block_dim_y = dim[1];
        header.block_dim_z = dim[2];
        header.log_block_length = log;
        header.padding = padding;
        header.frame_width = frame_width;
        header.frame_height = frame_height;
        header.voxel = voxel_type;

        VoxelCompressOptions opts;
        opts.width = frame_width;
        opts.height = frame_height;
        VoxelCompress cmp(opts);

        sv::Writer writer(output.c_str());
        writer.write_header(header);

        reader.startRead();
        while(!reader.isEmpty()){
            auto block = reader.getBlock();
            std::vector<std::vector<uint8_t >> packets;
//            {
//                std::string name = std::to_string(block.index[0])+"_"
//                                   +std::to_string(block.index[1])+"_"
//                                   +std::to_string(block.index[2])+".raw";
//                std::ofstream out(name,std::ios::binary);
//                out.write(reinterpret_cast<char*>(block.data.data()),block.data.size());
//                out.close();
//            }
            cmp.compress(block.data.data(),block.data.size(),packets);
            writer.write_packet(block.index,packets);
        }
    }
    catch (const std::exception& err)
    {
        std::cout<<err.what()<<std::endl;
        return -1;
    }
    return 0;
}