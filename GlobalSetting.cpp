#include "GlobalSetting.h"

GlobalSetting* GlobalSetting::m_instance = NULL;

GlobalSetting* GlobalSetting::instance()
{
	if (!m_instance) {
		m_instance = new GlobalSetting();
	}
	return m_instance;
}


GlobalSetting::GlobalSetting()
{
	readConfigFile();
}


GlobalSetting::~GlobalSetting()
{
}

bool GlobalSetting::readConfigFile()
{
	std::fstream cfgFile;
	cfgFile.open("./config.txt");
	if (!cfgFile.is_open()) {
		std::cout << "could not open this file!" << std::endl;
		return false;
	}
	char tmp[100];
	while (!cfgFile.eof()) {
		cfgFile.getline(tmp, 1000);
		std::string line(tmp);
		std::size_t pos = line.find("=");
		if (pos == std::string::npos) {
			std::cout << "no '=' find between key and value";
			return false;
		}
		std::string key = line.substr(0, pos);
		std::string value = line.substr(pos + 1);
		KV.insert(std::pair<std::string,std::string>(key, value));
	}
	return true;
}

