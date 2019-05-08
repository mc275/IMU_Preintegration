//
// Created by mc on 19-5-9.
//

#include <iostream>

#include <IMU_Preintegrator.h>
#include<fstream>

#include<opencv2/core/core.hpp>


void LoadEuRocIMUData(const std::string &strImuPath, std::vector<IMUData> &imu_datas);


int main(int argc, char **argv)
{

    std::vector<IMUData> imu_datas;

    std::string imu_file_path(argv[1]);
    LoadEuRocIMUData(imu_file_path, imu_datas);

    IMUPreintegrator IMUPreintegrator;

    std::ofstream output("../../../../log/new_pre.txt");

    for(int i=0; i<20; ++i)
    {
        IMUData imu_data = imu_datas[i];
        double dt = imu_datas[i+1].timestamp_ - imu_data.timestamp_;

        // 由于没有其他传感器融合，此处的IMU数据没有扣除bias
        IMUPreintegrator.Update(imu_data.gyr_, imu_data.acc_, dt);


        output << "Pre P " << i << std::endl;
        output << IMUPreintegrator.GetDeltaP() << std::endl;
        output << "Pre V " << i << std::endl;
        output << IMUPreintegrator.GetDeltaV() << std::endl;
        output << "Pre R " << i << std::endl;
        output << IMUPreintegrator.GetDeltaRot() << std::endl;
        output << "Pre Cov " << i << std::endl;
        output << IMUPreintegrator.GetCovPVRot() << std::endl;

        output << "Pre P jacobian gyr " << i << std::endl;
        output << IMUPreintegrator.GetPJacoBiasgyr() << std::endl;
        output << "Pre P jacobian acc " << i << std::endl;
        output << IMUPreintegrator.GetPJacoBiasacc() << std::endl;
        output << "Pre V jacobian gyr " << i << std::endl;
        output << IMUPreintegrator.GetVJacoBiasgyr() << std::endl;
        output << "Pre V jacobian acc " << i << std::endl;
        output << IMUPreintegrator.GetVJacoBiasacc() << std::endl;
        output << "Pre Rot jacobian acc " << i << std::endl;
        output << IMUPreintegrator.GetRotJacoBiasgyr() << std::endl;

    }


    return 0;

}



void LoadEuRocIMUData(const std::string &strImuPath, std::vector<IMUData> &imu_datas)
{

    std::ifstream fImus;
    fImus.open(strImuPath.c_str());
    imu_datas.reserve(30000);
    //int testcnt = 10;


    while (!fImus.eof())
    {
        std::string s;
        getline(fImus, s);
        if (!s.empty())
        {
            char c = s.at(0);
            if (c < '0' || c > '9')
                continue;

            std::stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            double data[10];    // timestamp, wx,wy,wz, ax,ay,az
            while (ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if (cnt == 7)
                    break;
                if (ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            // data[0]是时间戳。
            data[0] *= 1e-9;
            IMUData imudata(data[1], data[2], data[3],
                                       data[4], data[5], data[6], data[0]);
            imu_datas.push_back(imudata);

        }
    }
}