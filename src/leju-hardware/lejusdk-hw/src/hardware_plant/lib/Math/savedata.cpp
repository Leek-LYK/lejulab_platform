/* 1. First please add data folder parallel with 'build' folder
 *  2. Use remove(SavePATH); before saveVec3() function
 */
#include "savedata.h"

void saveVec3(const char *path, double timeSec, Vec3 v3)
{
    ofstream ws;
    ws.open(path, ios_base::app);
    if (ws.is_open())
    {
        ws << timeSec << "\t" << v3[0] << "\t" << v3[1] << "\t" << v3[2] << endl;
        ws.close();
    }
    else
    {
        cout << "Open file faile" << endl;
    }
}

void saveVx(const char *path, Eigen::VectorXd vx)
{
    ofstream ws;
    ws.open(path, ios_base::app);
    if (ws.is_open())
    {
        for (int i = 0; i < vx.size(); i++)
        {
            ws << vx[i] << "\t";
        }
        ws << "\n";
        ws.close();
    }
    else
    {
        cout << "Open file faile" << endl;
    }
}

Eigen::VectorXd readVx(const char *path)
{
    std::ifstream rs;
    rs.open(path);
    Eigen::VectorXd vx;
    if (rs.is_open())
    {
        std::vector<double> values;
        double value;
        while (rs >> value)
        {
            values.push_back(value);
        }
        rs.close();
        vx = Eigen::Map<Eigen::VectorXd>(values.data(), values.size());
    }
    else
    {
        std::cout << "Open file faile" << std::endl;
    }
    return vx;
}
