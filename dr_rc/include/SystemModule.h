
#include "DefineList.h"

int ParsingMainFuncArgs(int Arg_argc, char ** Arg_argv)
{
    // Initialization to set to 0
    memset(&StrMainFuncArgs, 0, sizeof(StrMainFuncArgs));


    // Assign Main Function Arguments
    StrMainFuncArgs.PtrArr_Args[0]  = DF_MAIN_FUNC_ARG_00;
    StrMainFuncArgs.PtrArr_Args[1]  = DF_MAIN_FUNC_ARG_01;
    StrMainFuncArgs.PtrArr_Args[2]  = DF_MAIN_FUNC_ARG_02;


    // Set Flags
    if(Arg_argc > 2)
    {
        int TempIndA;
        int TempIndB;

        for(TempIndA = 2; TempIndA < Arg_argc; TempIndA++)
        {
            for(TempIndB = 0; TempIndB < DF_MAIN_FUNC_ARGS_MAX; TempIndB++)
            {
                if(strcmp(Arg_argv[TempIndA],StrMainFuncArgs.PtrArr_Args[TempIndB]) == 0)
                {
                    StrMainFuncArgs.Flag_Args[TempIndB] = 1;
                    break;
                }
            }
        }
    }


    // Get LogFileName
    if(StrMainFuncArgs.Flag_Args[0] == 1) // Case of Activating Onboard Logging
    {
        printf("Type onboard log file name : ");
        scanf("%1023s",&StrMainFuncArgs.OnboardLogFileName[0]);
        sprintf(filename,"/home/ubuntu/bin/%s",StrMainFuncArgs.OnboardLogFileName);
    }

    return 1;
}

int System_Initialization(int argc, char** argv)
{

    if(ParsingMainFuncArgs(argc, argv) == 1)
    {
        printf("[DONE] Parsing main function arguments\n");
    }
    else
    {
        printf("[ERROR] 'DS_ParsingMainFuncArgs()'\n");
        return -1;
    }

}

