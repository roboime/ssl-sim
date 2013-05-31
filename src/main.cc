/*
 * Small Size League Simulator (Experimental) (c) RoboIME
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it freely, 
 * subject to the following restrictions:
 *
 * This is a modified version from the DynamicControlDemo shipped with bullet 2.81
 */


#include "sslsim.h"
#include "GlutStuff.h"

int main(int argc,char* argv[])
{
        SSLSim demoApp;

        demoApp.initPhysics();

        return glutmain(argc, argv,640,480,"Small Size League Simulator (Experimental)",&demoApp);
}
