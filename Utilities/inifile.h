/**
 * @file
 * @brief initialization file read and write API 
 * @author Deng Yangjun
 * @date 2007-1-9
 * @version 0.1
 */
 
#ifndef INI_FILE_H_
#define INI_FILE_H_

#ifdef __cplusplus
extern "C"
{
#endif

int read_profile_string( const char *section, const char *key,char *value, int size,const char *default_value, const char *file);
int read_profile_int( const char *section, const char *key,int default_value, const char *file);
float read_profile_float( const char *section, const char *key,float default_value, const char *file);
int write_profile_string( const char *section, const char *key,const char *value, const char *file);

#ifdef __cplusplus
}; //end of extern "C" {
#endif

#endif //end of INI_FILE_H_

