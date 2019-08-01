/*
 * shell.h
 *
 *  Created on: 27 sept. 2019
 *      Author: Ross
 */

#ifndef SHELL_H_
#define SHELL_H_

//#define NULL					0
/*#define SHELL_MAX_LINE_LENGTH	16
#define SHELL_MAX_ARGUMENTS		4
#define SHELL_PROMPT			"STM32> "
#define FW_VERSION				"0.4.0.0"
#define OS_VERSION				"9.0.0"
#define CR						"\r\n"
*/
//	Function prototype for the action executed by the shell
typedef void (* shellcmd_t)(int argc, char *argv[]);
//
//	Shell Command Data type:
//	Struct composed by the name of the command and a function prototype for the action to execute
//
typedef struct
{
	const char *sc_name;
	shellcmd_t  sc_function;
} ShellCommand_t;

//
//	Shell Configuration data type (don't think it is used)
//
typedef struct
{
	const ShellCommand_t *sc_command;
} ShellConfig_t;


//	Configuration data structure
static ShellConfig_t ShellConfig;

//
//	Shell Class
//
class cShell {
private:
	char * vStrtok(char *str, const char *delim, char **saveptr);
	//	Function prototype for the action executed by the shell
//	typedef void (* shellcmd_t)(int argc, char *argv[]);
	//	Function to execute the command
	int vCmdExec(const ShellCommand_t *scp, char *name, int argc, char *argv[]);

public:
	cShell();
	virtual ~cShell();
	//
	//	Shell main task
	//
	void ShellTask(void *p, char *line);
	void vShellThread (void *p);
//	void avrPrintf(const char * str);
};

//extern cShell *shell;

/*
//
//	Actions to be executed by the shell defined in different files
//
void vpidToggle(int argc, char *argv[]);			//	Toggle ON/OFF the PID
void vpidSet(int argc, char *argv[]);				//	Set PID coefficients
void vpidGet(int argc, char *argv[]);				//	Get PID coefficients
void vpidGetError(int argc, char *argv[]);			//	Get the error out of the PID
*/
//
//	Local Actions of the Shell
//
void vSendACK(int argc, char *argv[]);					//	Send back a ACK to the PC
void vCmdInfo(int argc, char *argv[]);					//	Get back the info about the FW revision
void vCmdSystime(int argc, char *argv[]);				//	Command system time (not implemented)
void vUsage(char *str);
void vListCommands(ShellCommand_t *scp);
//
//	Actions to be executed by the shell defined in different files
//
extern void ADCInit(int argc, char *argv[]);			//	Initialize the ADC
extern void getADC_Result(int argc, char *argv[]);		//	Read from the ADC
extern void vPwmInit(int argc, char *argv[]);			//	Initialize a PWM
extern void vPwmStart(int argc, char *argv[]);			//	Start a PWM
extern void vPwmStop(int argc, char *argv[]);			//	Stop a PWM
extern void vPwmSet(int argc, char *argv[]);			//	Set the level of a PWM
//
//	Definition of the shell commands:
//	This data structure links the command to the corrisponding action function
//
static ShellCommand_t ShellCommand[] =
{
	{
		"get_ACK",	vSendACK
	},
	{
		"pwm_init", vPwmInit
	},
	{
		"pwm_start", vPwmStart
	},
	{
		"pwm_stop", vPwmStop
	},
	{
		"pwm_set", vPwmSet
	},
	{
		"adc_init", ADCInit
	},
	{
		"adc_get", getADC_Result
	},
	{
		NULL, NULL
		//(char *)0, (void **)0
	}
} ;



#endif /* SHELL_H_ */
