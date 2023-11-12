// small hacked program to compile all possible combinations of build settings
// WARNING - MADNESS !!! A LOOOOOOT OF PERMUTATIONS EXIST! WHAT A HUGE WASTE OF ENERGY! 768 per config without demos and test option toggled
#include <stdio.h>
#include <string.h>
#include <string>
#include <map>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

#define STRINGIZE(x)     #x
#define COUNT_OF(a) ( sizeof(a)/sizeof((a)[0]) )

#define BOOL_OPTION(o,content) \
  for(int i_##o=0;i_##o<2;++i_##o) { \
      if(i_##o==0) \
        options[STRINGIZE(o)]="ON"; \
      else \
        options[STRINGIZE(o)]="OFF"; \
      content \
  }

#define FIXED_OPTION(o,value,content) \
  int i_##o=value; \
  if(i_##o==0) \
    options[STRINGIZE(o)]="ON"; \
  else \
    options[STRINGIZE(o)]="OFF"; \
  content

// filter:
// - ODE_NO_BUILTIN_THREADING_IMPL==OFF && ODE_WITH_OU==OFF (OU Atomics must be enabled for this to compile)
// - both mesh libraries off
static bool is_compatible_combination(const std::map<std::string,std::string>& options)
{
  for(auto it = options.begin(); it!= options.end(); ++it)
  {
    if(it->first == "ODE_NO_BUILTIN_THREADING_IMPL" && it->second=="OFF")
    {
      for(auto it2 = options.begin(); it2!= options.end(); ++it2)
      {
        if(it2->first == "ODE_WITH_OU" && it2->second=="OFF")
          return false;
      }
    }
  }
  for(auto it = options.begin(); it!= options.end(); ++it)
  {
    if(it->first == "ODE_WITH_GIMPACT" && it->second=="OFF")
    {
      for(auto it2 = options.begin(); it2!= options.end(); ++it2)
      {
        if(it2->first == "ODE_WITH_OPCODE" && it2->second=="OFF")
          return false;
      }
    }
  }
  return true;
}

/* to print correct amount of permutations representing as a graph and counting edges would be required */
#define INVOKE_BUILD(compiler,buildType,start,...) \
  if(currOption>=start)\
  {\
    if(is_compatible_combination(options))\
    {\
      std::string cmd;\
      cmd += "cmake ";\
      cmd += "-DCMAKE_BUILD_TYPE=" + std::string(buildType) + " ";\
      cmd += "-DCMAKE_EXPORT_COMPILE_COMMANDS=TRUE ";\
      cmd += "-DCMAKE_TOOLCHAIN_FILE=" + std::string(compiler) + " ";\
      for(auto it = options.begin(); it!= options.end(); ++it)\
        cmd += "-D" + it->first + "=" + it->second + " ";\
      cmd += "-B cpBuild ";\
      cmd += "..";\
      cmd += " && cmake --build ./cpBuild --parallel";\
      (void)fprintf(stdout,"Config: \"%s\"\n",cmd.c_str());\
      std::string cmdInfo;\
      for(auto it = options.begin(); it!= options.end(); ++it)\
        cmdInfo += "-D" + it->first + "=" + it->second + " ";\
      (void)fprintf(stderr,"Config %i: \"%s\"\n",currOption++,cmdInfo.c_str());\
      (void)fflush( stdout );\
      (void)fflush( stderr );\
      int returnCode = system(cmd.c_str());\
      if(returnCode != 0)\
      {\
        (void)fprintf(stderr,"Build failed at config %i: %s\n",(currOption-1),cmd.c_str());\
        (void)fflush( stderr );\
        return returnCode;\
      }\
    }\
    else\
    {\
      (void)fprintf(stderr,"Skipping %i - incompatible combination\n",currOption++);\
    }\
  }\
  else \
    currOption++

static int invoke(const char* compiler, const char* buildType, const char* c_std, const char* cpp_std, int start)
{
  (void)c_std; (void)cpp_std; // not implemented
  int currOption=1;
  std::map<std::string,std::string> options;
  (void)fprintf(stderr,"Config %s with compiler at %s\n",buildType,compiler);

  // BOOL_OPTION(ODE_WITH_DEMOS,
  // BOOL_OPTION(ODE_WITH_TESTS,
  BOOL_OPTION(ODE_NO_BUILTIN_THREADING_IMPL,
  BOOL_OPTION(ODE_NO_THREADING_INTF,
  BOOL_OPTION(ODE_WITH_OU,
  BOOL_OPTION(ODE_16BIT_INDICES,
  BOOL_OPTION(ODE_WITH_LIBCCD,
  BOOL_OPTION(ODE_DOUBLE_PRECISION,
  BOOL_OPTION(ODE_NO_TRIMESH,
    // OPCODE
    {
    FIXED_OPTION(ODE_WITH_OPCODE ,1,
    FIXED_OPTION(ODE_WITH_GIMPACT,0,
      BOOL_OPTION(ODE_OLD_TRIMESH,
        INVOKE_BUILD( compiler,
                      buildType,
                      start,
                      ODE_WITH_DEMOS,
                      ODE_WITH_TESTS,
                      ODE_NO_BUILTIN_THREADING_IMPL,
                      ODE_NO_THREADING_INTF,
                      ODE_WITH_OU,
                      ODE_NO_TRIMESH,
                      ODE_16BIT_INDICES,
                      ODE_WITH_LIBCCD,
                      ODE_DOUBLE_PRECISION,
                      ODE_WITH_OPCODE,
                      ODE_OLD_TRIMESH
          );
      )
    ))
    }
    // GIMPACT
    {
    FIXED_OPTION(ODE_WITH_OPCODE ,0,
    FIXED_OPTION(ODE_WITH_GIMPACT,1,
      INVOKE_BUILD( compiler,
                    buildType,
                    start,
                    ODE_WITH_DEMOS,
                    ODE_WITH_TESTS,
                    ODE_NO_BUILTIN_THREADING_IMPL,
                    ODE_NO_THREADING_INTF,
                    ODE_WITH_OU,
                    ODE_NO_TRIMESH,
                    ODE_16BIT_INDICES,
                    ODE_WITH_LIBCCD,
                    ODE_DOUBLE_PRECISION,
                    ODE_WITH_GIMPACT);
    ))
    }
  )))))))
  // ));
  return 0;
}

int main(int argc, char** argv)
{
  int start=0;
  std::string compiler;
  std::string buildType = "Release";
  for(int i=1;i<argc;++i)
  {
    if(strcmp(argv[i],"-f")==0 && i<argc)
    {
      start = atoi(argv[i+1]);
      printf("Start from %i\n",start);
    }
    else if(strcmp(argv[i],"-c")==0 && i<argc)
    {
      compiler = argv[i+1];
      // toolchain has to be empty or end with trailing slash
      if(!compiler.empty() && compiler.back()!='/')
      {
        compiler+='/';
      }
    }
    else if(strcmp(argv[i],"-t")==0 && i<argc)
    {
      buildType = argv[i+1];
    }
  }

  // const char* compilerList[]      = { "" }; //, "/usr/bin/", "/path-to-mingw-toolchain/bin/" };
  // const char* buildTypes[]    = { "Release"/*, "Debug"*/ };
  const char* c_standards[]   = { "c89"/*, "c99"   , "c11"  ,          "c17"*/ /* not gcc4 */ };
  const char* cpp_standards[] = {        "c++98" /*, "c++11", "c++14", "c++17", "c++20", "c++23"*/ };

  // from https://stackoverflow.com/questions/22541158/redirect-std-output-of-a-c-function-to-null
  (void)fflush( stdout );                            // make sure everything is written to stdout
  int savefd = dup( 1 );                       // dup stdout for restore
  int outfd = creat( "compileLog.txt", 0666 ); // create an output file
  dup2( outfd, 1 );                            // dup that fd to stdout

  // for (int c = 0; c < COUNT_OF(compiler); ++c)
    // for (int b = 0; b < COUNT_OF(buildTypes); ++b)
        for (size_t s = 0; s < COUNT_OF(c_standards); ++s)
            for (size_t s2 = 0; s2 < COUNT_OF(cpp_standards); ++s2)
              invoke(compiler.c_str(),buildType.c_str(),c_standards[s],cpp_standards[s2],start);

  (void)fflush( stdout );                      // make sure everything is written to stdout
  close( outfd );
  dup2( savefd, 1 );

  printf("Done\n");
  return 0;
}
