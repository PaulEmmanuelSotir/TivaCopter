/*
 * CmdLineWarper.h
 */

#ifndef CMDLINEWARPER_H_
#define CMDLINEWARPER_H_

//----------------------------------------
// Command line warper task
//----------------------------------------
void SubscribeWarperCmds(void);

//----------------------------------------
// UART console commands for quadcopter
// API warper.
//----------------------------------------
void I2CSelect_cmd(int argc, char *argv[]);
void I2CRegRead_cmd(int argc, char *argv[]);
void I2CRegWrite_cmd(int argc, char *argv[]);
void I2CRegReadModifyWrite(int argc, char *argv[]);
void I2CWrite_cmd(int argc, char *argv[]);

#endif /* CMDLINEWARPER_H_ */
