/*
 * JSONCommunication.h
 */

#ifndef JSON_COMMUNICATION_H_
#define JSON_COMMUNICATION_H_

#include <stdint.h>
#include <stdbool.h>
#include <xdc/std.h>

#include "Utils/UARTConsole.h"

#ifndef MAX_DATASOURCE_COUNT
#define MAX_DATASOURCE_COUNT			10
#endif

#ifndef MAX_DATAINPUT_COUNT
#define MAX_DATAINPUT_COUNT				2
#endif

#ifndef INPUT_JSON_BUFFER_SIZE
#define INPUT_JSON_BUFFER_SIZE			512
#endif

#ifndef INPUT_JSON_TOKEN_NUM
#define INPUT_JSON_TOKEN_NUM			128
#endif

#ifndef MAX_DATA_COUNT
#define MAX_DATA_COUNT					32
#endif

//----------------------------------------
// Data accessor function typedef used to
// get string data array from datasources.
//----------------------------------------
typedef char** (*DataValuesGetAccessor)(void);
typedef void (*DataValuesSetAccessor)(char**);

//------------------------------------------
// A structure gathering informations about
// a JSON data source.
// User should use API functions instead of
// modifing directly members of this struct.
//------------------------------------------
typedef struct
{
	// Datasource name
	const char* name;
	// Data names table
	const char** keys;
	// Data member count (length of arrays)
	uint32_t dataCount;
	// Boolean indicating wether if the datasource should send its data or not.
	bool enabled;
	// Period of the data source data sending in RTOS clock ticks (0 means not periodic)
	uint32_t period;
	// If the datasource is periodic, this handle keep track of the datasource clock.
	Clock_Handle clock;
	DataValuesGetAccessor dataAccessor;
	// Flag used to indicate to sending task that this data source need to send its data
	bool sendNowFlag;
} JSONDataSource;

//-------------------------------------------
// A structure typedef gathering informations
// about a JSON data input.
// User should use API functions instead of
// modifing directly members of this struct.
//-------------------------------------------
typedef struct
{
	// Datas input name
	const char* name;
	// Data names table
	const char** keys;
	// Data member count (length of arrays)
	uint32_t dataCount;
	DataValuesSetAccessor dataAccessor;
} JSONDataInput;

//----------------------------------------
// UART console commands for JSON
// communication.
//----------------------------------------
void JSON_list_sources_cmd(int argc, char *argv[]);
void JSON_enable_cmd(int argc, char *argv[]);
void JSON_disable_cmd(int argc, char *argv[]);
void JSON_start_cmd(int argc, char *argv[]);
void JSON_enable_programatic_access_cmd(int argc, char *argv[]);
void JSON_disable_programatic_access_cmd(int argc, char *argv[]);

//---------------------------------------------
// Subscribe data source:
// Creates a data source and get it from static
// 'JSONDataSources' array.
// The 'keys' array should contain 'DataCount'
// names of the data fields provided by the
// datasource.
//---------------------------------------------
JSONDataSource* SubscribeJSONDataSource(const char* name,const char* keys[], uint32_t dataCount);
JSONDataSource* SubscribeJSONDataSource2(const char* name,const char* keys[], uint32_t dataCount, bool enabled);


//--------------------------------------------
// Subscribe periodic data source:
// Creates a periodic data source and get it
// from static 'JSONDataSources' array.
// The 'keys' array should contain 'DataCount'
// names of the data fields provided by the
// datasource.
// 'period' is the period of sending in RTOS
// clock ticks.
//--------------------------------------------
JSONDataSource* SubscribePeriodicJSONDataSource(const char* name, const char* keys[], uint32_t dataCount, uint32_t period, DataValuesGetAccessor DataAccessor);
JSONDataSource* SubscribePeriodicJSONDataSource2(const char* name, const char* keys[], uint32_t dataCount, uint32_t period, DataValuesGetAccessor dataAccessor, bool enabled);

//--------------------------------------------
// Unsubscribe JSON data source:
// Unsubscribes given data source.
// Return false if the given datasource wasn't
// subscribed, returns true otherwise.
//--------------------------------------------
bool UnsubscribeJSONDataSource(JSONDataSource* datasource);

//--------------------------------------------
// Send data:
// Send specified data corresponding to given
// datasource ('values' array should have the
// same size as datasource's 'keys' array).
// Returns true if json data have been
// successfully sent, returns false otherwise.
//--------------------------------------------
bool SendJSONData(JSONDataSource* ds, char* values[]);

//------------------------------------------
// Periodic data sending task:
// Sends JSON data from periodic datasources
// in a low priority task.
//------------------------------------------
void PeriodicJSONDataSendingTask(void);

//-----------------------------------------
// Periodic data sending software interrupt
//-----------------------------------------
void PeriodicJSONDataSendingSwi(UArg dataSource);

//--------------------------------------------
// Subscribe JSON data input
// Subscribe to incomming data corresponding
// to given input name and key.
// The 'keys' array should contain 'DataCount'
// names of the data fields that will be
// populated as soon as corresponding JSON
// data is received.
//--------------------------------------------
JSONDataInput* SubscribeJSONDataInput(char* name, const char* keys[], uint32_t dataCount, DataValuesSetAccessor dataAccessor);

//--------------------------------------------
// Unsubscribe JSON data input:
// Unsubscribes given data input.
// Returns false if the given data input wasn't
// subscribed, returns true otherwise.
//--------------------------------------------
bool UnsubscribeJSONDataInput(JSONDataInput* datainput);

#endif /* JSON_COMMUNICATION_H_ */
