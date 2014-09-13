/*
 * JSONCommunication.h
 */

#ifndef JSON_COMMUNICATION_H_
#define JSON_COMMUNICATION_H_

#include <stdint.h>
#include <stdbool.h>
#include <xdc/std.h>

#include "Utils/UARTConsole.h"

//----------------------------------------
// Data accessor function typedef used to
// get string data array from datasources.
//----------------------------------------
typedef char** (*DataValuesAccessor)(void);

//------------------------------------------
// A structure gathering informations about
// a JSON data source.
// User should use API functions instead of
// modifing directly members of this struct.
//------------------------------------------
typedef struct
{
	// Datasource name
	char* name;
	// Data names table
	const char** keys;
	// Data member count (length of arrays)
	uint32_t dataCount;
	// Boolean indicating wether if the datasource should send its data or not.
	bool enabled;
	// Period of the data source data sending in RTOS clock ticks (0 means not periodic)
	uint32_t period;
	DataValuesAccessor dataAccessor;
	// Flag used to indicate to sending task that this data source need to send its data
	bool sendNowFlag;
} JSONDataSource;

//----------------------------------------
// UART console commands for JSON
// communication.
//----------------------------------------
void JSON_list_sources_cmd(int argc, char *argv[]);
void JSON_enable_cmd(int argc, char *argv[]);
void JSON_disable_cmd(int argc, char *argv[]);
void JSON_start_cmd(int argc, char *argv[]);

//--------------------------------------------
// Suscribe data source:
// Creates a data source and add it to dynamic
// 'dataSources' array.
// The 'keys' array should contain 'DataCount'
// names of the data fields provided by the
// datasource.
//--------------------------------------------
JSONDataSource* SuscribeJSONDataSource(char* name,const char* keys[], uint32_t dataCount);

//--------------------------------------------
// Suscribe periodic data source:
// Creates a periodic data source and add it
// to dynamic 'dataSources' array.
// The 'keys' array should contain 'DataCount'
// names of the data fields provided by the
// datasource.
// 'period' is the period of sending in RTOS
// clock ticks.
//--------------------------------------------
JSONDataSource* SuscribePeriodicJSONDataSource(char* name, const char* keys[], uint32_t dataCount, uint32_t period, DataValuesAccessor DataAccessor);

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

#endif /* JSON_COMMUNICATION_H_ */
