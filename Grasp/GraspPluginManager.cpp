/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "GraspController.h"
#include "GraspPluginManager.h"
#include <cnoid/ExtensionManager>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/App>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/Config>	/* modified by qtconv.rb 0th rule*/

#include <map>
#include <vector>
#include <ostream>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/regex.hpp>
#include <boost/tokenizer.hpp>
#include <boost/format.hpp>
#include <boost/version.hpp>

#include <QLibrary>	/* modified by qtconv.rb 1st rule*/

#include <iostream>


using namespace std;
using namespace boost;

#ifdef WIN32
static const string DLL_SUFFIX = "dll";
#else
static const string DLL_SUFFIX = "so";  /* modified by hand 8th rule */
#endif

#ifdef G_OS_WIN32
static const string DLL_PREFIX = "";
static const char* PATH_DELIMITER = ";";
# ifdef EXCADE_DEBUG
static const string DEBUG_SUFFIX = "d";
# else
static const string DEBUG_SUFFIX = "";
# endif
#elif defined WIN32
static const string DLL_PREFIX = "";
static const char* PATH_DELIMITER = ":";
#ifdef _DEBUG
static const string DEBUG_SUFFIX = "d";
#else
static const string DEBUG_SUFFIX = "";
#endif
#else
static const string DLL_PREFIX = "lib";
static const char* PATH_DELIMITER = ":";
static const string DEBUG_SUFFIX = "";
#endif


using namespace grasp;


GraspPluginManager::GraspPluginManager()
    : os(cnoid::MessageView::mainInstance()->cout())
{
    pluginNamePattern = DLL_PREFIX + "Grasplot.+" + DEBUG_SUFFIX + "\\." + DLL_SUFFIX;	/* modified by qtconv.rb 8th rule*/
    hiddenFileDirPattern = "^\\..+";
		getGrasplotFunc = NULL;
}


GraspPluginManager::~GraspPluginManager()
{
}

void GraspPluginManager::scanPluginFiles(const std::string& pathString)
{
    filesystem::path pluginPath(pathString);

    if(filesystem::exists(pluginPath)){

        if(filesystem::is_directory(pluginPath)){
            filesystem::directory_iterator end;
            for(filesystem::directory_iterator it(pluginPath); it != end; ++it){
                const filesystem::path& filepath = *it;
#if (BOOST_VERSION<104600)
                if(!regex_match(filepath.leaf(), hiddenFileDirPattern))
#else
                if(!regex_match(filepath.filename().string().c_str(), hiddenFileDirPattern))
#endif
		{
                    scanPluginFiles(filepath.string());
                }
            }
        } else {

#if (BOOST_VERSION<104600)
            if(regex_match(pluginPath.leaf(),pluginNamePattern))
#else
            if(regex_match(pluginPath.filename().string().c_str(),pluginNamePattern))
#endif
	    {

                PluginMap::iterator p = pathToPluginInfoMap.find(pathString);
                if(p == pathToPluginInfoMap.end()){

                    PluginInfoPtr info(new PluginInfo);

                    info->pathString = pathString;
                    info->plugin = 0;
                    info->status = GraspPluginManager::NOT_LOADED;

                    pluginInfoList.push_back(info);
                    pathToPluginInfoMap[pathString] = info;
                }
            }
        }
    }
}

void* GraspPluginManager::loadGrasplotPlugin(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm, string className)
{
/* modified by hand 9th rule*/
//    if(!Glib::Module::get_supported()){
//        os << "Loading plugin dlls is not supported on this platform." << endl;
//        return 0;
//    }

	for(int index=0; index < pluginInfoList.size(); index++){

		PluginInfoPtr info = pluginInfoList[index];

		if(info->status == GraspPluginManager::NOT_LOADED){
			os << string("Loading plugin file \"") + info->pathString + "\"" << endl;

			info->dll.setFileName(info->pathString.c_str());	/* modified by qtconv.rb 10th rule*/
//			info->dll.setLoadHints (QLibrary::ExportExternalSymbolsHint |  QLibrary::LoadArchiveMemberHint | QLibrary::ResolveAllSymbolsHint); 
			if(!(info->dll.load())){ 		/* modified by qtconv.rb 11th rule*/
				info->status = GraspPluginManager::INVALID;
				os << info->dll.errorString().toStdString() << endl;	/* modified by qtconv.rb 12th rule*/
			}else{
				info->status = GraspPluginManager::LOADED;
			}
		}
		if(info->status == GraspPluginManager::LOADED){
	 		/* modified by hand 13th rule*/
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
			void* symbol = info->dll.resolve(info->pathString.c_str(), string("getGrasplot" + className).c_str());
#else
			QFunctionPointer symbol = info->dll.resolve(info->pathString.c_str(), string("getGrasplot" + className).c_str());
#endif
			if(!symbol){
				return 0;
			} else {
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
				info->plugin = symbol;
#else
				GrasplotEntry getCnoidPluginFunc = (GrasplotEntry)(symbol);
				info->plugin = getCnoidPluginFunc(body, base, palm);
#endif
				/*GrasplotEntry*/ getGrasplotFunc = (GrasplotEntry)(symbol);

				info->name =  body->name()  + ":" + className ;
				if(symbol) os << "Loading Plugins " << info->name << endl;
				return (*getGrasplotFunc)(body,  base, palm );
			}
		}
	}
	return 0;
}
