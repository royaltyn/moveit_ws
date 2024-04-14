#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>
#include <visual_grab/LocalPointMsg.h> 
#include <string>

#include<fstream>
#include <cstring>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


using namespace std;

//去掉首部空格
string& ClearHeadSpace(string &str)   
{  
    if (str.empty())   
    {  
        return str;  
    }  
 
    str.erase(0,str.find_first_not_of(" "));  
    //str.erase(str.find_last_not_of(" ") + 1);  
    return str;  
} 

std::string& replace_all_distinct(std::string&  str,const std::string&   old_value,const std::string& new_value)
{
    for(std::string::size_type   pos(0);   pos!=std::string::npos;   pos+=new_value.length())
    {
        if((pos=str.find(old_value,pos))!=std::string::npos)
        {
            str.replace(pos,old_value.length(),new_value);
        }
        else
        {
            break;
        }
    }
    return  str;
}
/*
作用： 以字符delim 分割字符串s
参数:delim  子字符，如'\n'
    s 从文件中读出来的字符串
*/
std::vector<std::string> StringSplit2(const  std::string& s, const std::string& delim)
{
    std::vector<std::string> elems;
    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();
    if (delim_len == 0) return elems;
    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }
    return elems;
}

string ReadFile(string& filePath)
{
    string strData="";
    const char * p8File = (char*)filePath.c_str();

    FILE *fp;
    fp = fopen(p8File,"r");
    if(fp == NULL)
    {
      cout<<"cannot read file, open error"<<endl;
      return "false";
    }

    fseek(fp,0,SEEK_END);//将文件内部的指针指向文件末尾
    long lsize=ftell(fp);//获取文件长度，（得到文件位置指针当前位置相对于文件首的偏移字节数）
    rewind(fp);//将文件内部的指针重新指向一个流的开头
    //fclose(fp);

    char *pData= new char[lsize+1];
    memset(pData,0,lsize+1);//将内存空间都赋值为‘\0’

    int result=fread(pData,1,lsize,fp);//将pfile中内容读入pread指向内存中
    fclose(fp);
    strData = std::string(pData);

    if(pData)
    {
        delete[] pData;
        pData = NULL;
    }
    return strData;
}



string ChangeFile(string& strData,string& name, geometry_msgs::PointStamped& point)
{
    string strLine="";
    std::string oneLine="";
    
    //建议去掉行前的空格 和空的行
    
    // 以\n 分行
    std::vector<std::string> strList = StringSplit2(strData,   "\n");

    //搜索每一行
    int i=0;
    string num="";
    bool hasFind=false;
    for(;i<strList.size();i++)
    {   
        //去掉字符串头部空格，然后返回     
        oneLine = ClearHeadSpace(strList[i]);
        oneLine = replace_all_distinct(oneLine,"\r", "");
        //如果是空行，直接跳过
        if(oneLine.size()<1)
        {
            cout<<"this is empty line,continue"<<endl;
            continue;
        }
        
        //还没找到要修改的行时，
        if(!hasFind)
        {
            //查找一下，这一行是否是需要修改的行, 如果是则修改并结束循环
            if(oneLine.size() >= name.size() && oneLine.find(name) != std::string::npos)
            {
                
                //如果找到，有这样地域名的一样，就以= 号分割，将前面编号取出来
                std::vector<std::string> strList2 = StringSplit2(oneLine,  "=");                
                //记录一下编号，后面可能会用到
                
                num=strList2[0];
                //再重新组成一行
                strLine += strList2[0]+string("=")+to_string(point.point.x) +string(",")+ \
                           to_string(point.point.y)+ string(",") + to_string(point.point.z)+string(",")+ name + "\r\n";
                           
                hasFind=true;
                
                //break;
            }
            else //不是就记录这一行内容，继续下一行
            {
                //以= 号分割，将前面编号取出来
                std::vector<std::string> strList2 = StringSplit2(oneLine,   "=");
                //记录一下编号，后面可能会用到
                num=strList2[0];
                strLine +=oneLine + "\r\n";
                //continue;
            }
        }
        else //如果已经修改行了，就不再去查找了，把其他内容加在后面，保持不变就行
        {
            //以= 号分割，将前面编号取出来
            std::vector<std::string> strList2 = StringSplit2(oneLine,   "=");
            //记录一下编号，后面可能会用到
            num=strList2[0];
            strLine +=oneLine + "\r\n";
        }
        
    }
    //如果到末尾都没有找到要修改的行，则证明该行是新的，需要为文件尾部添加
    if(!hasFind)
    {
        //将num 的string类 变成int 类， 然后+1 , 再组成新的行。
        int newNum=atoi(num.c_str())+1;
        strLine += to_string(newNum)+string("=")+to_string(point.point.x) +string(",")+ \
                       to_string(point.point.y)+ string(",") + to_string(point.point.z)+string(",")+ name + "\r\n";
    }
    strList.clear();
    hasFind=false;
    num="";
    return strLine;
}

int writeToYamlFile(string& filePath,string& strData)
{
    const char * p8File = (char*)filePath.c_str();
    const char* pChangeData = strData.c_str();
    //把修改后的内容写到新的文件中
    int fd = open(p8File,  O_CREAT | O_WRONLY | O_TRUNC,0644);
    if(fd<0)
    {
       // LOGI("%s: vWriteDataToFile Create error",TAG);
        cout<<"in writeToFile open file error"<<endl;
        return false;
    }
    int ret = write(fd, pChangeData, strData.size());
    close(fd);
    return ret;
}

void updateInfoToFile(string& filePath,string& name, geometry_msgs::PointStamped& point)
{
    string strData=ReadFile(filePath);
    string changeFile=ChangeFile(strData,name,point);
    int ret=writeToYamlFile(filePath,changeFile);
    if(!ret)
    {
        cout<<"cannot write to file ,open file error"<<endl;
    }
}

void recordPoseCallback(const visual_grab::LocalPointMsg& msg)
{
    //将点转换到map 坐标系下
    
    string name=msg.localName;
    
    geometry_msgs::PointStamped point =msg.pose;
    //cout<<"get info: "<<name <<","<<point.point.x <<","<<point.point.y<<","<<point.point.z<<endl;
    
    //加入或更新到race.txt 文件
    string filePath="/home/eaibot/moveit_ws/src/moveit/visual_grab/config/race.txt";
    
    
    updateInfoToFile(filePath,name,point);
    cout<<"have add info: "<<name <<","<<point.point.x <<","<<point.point.y<<","<<point.point.z<<endl;
        
}

int main(int argc, char**argv)
{
    ros::init(argc,argv,"record_pose_to_file");
    ros::NodeHandle n;
    ros::Rate loop(1);
    
    ros::Subscriber offset_save_sub=n.subscribe("/record_pose",1,recordPoseCallback);
    
    loop.sleep();
    ros::spin();
    return 0;
}
