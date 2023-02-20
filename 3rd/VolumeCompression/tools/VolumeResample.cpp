//
// Created by wyz on 2021/4/7.
//
#include<cmdline.hpp>
#define VoxelResampleIMPL
#include<VoxelCompression/utils/VoxelResample.h>

int main(int argc,char** argv)
{
    auto system_memory_gb = get_system_memory() / 1024 /*kb*/ / 1024 /*mb*/ / 1024 /*gb*/;

    cmdline::parser cmd;
    cmd.add<std::string>("in",'i',"input file path",true);
    cmd.add<std::string>("in_format",'f',"input data format: raw/tif",true,"raw",cmdline::oneof<std::string>("raw","tif"));
    cmd.add<std::string>("prefix",'p',"prefix for input is tif format",false);
    cmd.add<std::string>("out",'o',"output file name",true);
    cmd.add<std::string>("out_dir",'d',"output file directory",false,".");
    cmd.add<int>("raw_x",'x',"x resolution of raw volume data",true);
    cmd.add<int>("raw_y",'y',"y resolution of raw volume data",true);
    cmd.add<int>("raw_z",'z',"z resolution of raw volume data",true);
    cmd.add<int>("memory_limit",'m',"maximum memory limit in GB",false,system_memory_gb/2);
    cmd.add<std::string>("method",'t',"resample method: average/max",false,"max",cmdline::oneof<std::string>("average","max"));

    cmd.parse_check(argc,argv);
    auto input_path=cmd.get<std::string>("in");
    auto input_format=cmd.get<std::string>("in_format")=="raw"?VolumeResampler::InputFormat::RAW:VolumeResampler::InputFormat::TIF;
    auto prefix=cmd.get<std::string>("prefix");
    auto output_name=cmd.get<std::string>("out");
    auto output_dir=cmd.get<std::string>("out_dir");
    auto output_path=output_dir+"/"+output_name;
    auto raw_x=cmd.get<int>("raw_x");
    auto raw_y=cmd.get<int>("raw_y");
    auto raw_z=cmd.get<int>("raw_z");
    auto mem_limit=min(cmd.get<int>("memory_limit"),system_memory_gb*3/4);
    auto method=cmd.get<std::string>("method");
    auto resample_method = method=="max"?VolumeResampler::ResampleMethod::MAX:VolumeResampler::ResampleMethod::AVG;
    try {
      VolumeResampler volume_resampler(input_path, input_format, prefix,
                                       output_path, raw_x, raw_y, raw_z,
                                       mem_limit, resample_method);
      volume_resampler.print_args();
      volume_resampler.start_task();
    }
    catch (const std::exception& err) {
      std::cout<<err.what()<<std::endl;
    }
    return 0;
}
