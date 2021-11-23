#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <map>

class GlobalSetting
{
public:
	GlobalSetting();
	~GlobalSetting();

	static GlobalSetting* instance();
	bool readConfigFile();
	std::map<std::string,std::string> KV;

private:
	static GlobalSetting* m_instance;

};


