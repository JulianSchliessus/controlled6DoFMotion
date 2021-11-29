/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | www.openfoam.com
     \\/     M anipulation  |
-------------------------------------------------------------------------------
    Copyright (C) 2011-2016 OpenFOAM Foundation
    Copyright (C) 2020 OpenCFD Ltd.
-------------------------------------------------------------------------------
License
    This file is part of OpenFOAM.

    OpenFOAM is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenFOAM is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
    for more details.

    You should have received a copy of the GNU General Public License
    along with OpenFOAM.  If not, see <http://www.gnu.org/licenses/>.

\*---------------------------------------------------------------------------*/

#include "controlled6DoFMotion.H"
#include "addToRunTimeSelectionTable.H"
#include "Tuple2.H"
#include "IFstream.H"
#include "interpolateSplineXY.H"
#include "unitConversion.H"

#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#define MAXBUFFERSIZE 150
#define SOCKET_PATH_SIZE 20

// Function designed for chat between client and server.
std::string func(int sockfd)
{

	printf("\nReading...\n");
	char buff[MAXBUFFERSIZE];
	std::string data;
	// infinite loop for chat
	bzero(buff, MAXBUFFERSIZE);
	std::cout << buff << std::endl;

	// read the message from client and copy it in buffer
	read(sockfd, buff, sizeof(buff));
	std::cout << buff << std::endl;
	// print buffer which contains the client contents
	printf("From client: %s\t To client : ", buff);
	data = buff;

	bzero(buff, MAXBUFFERSIZE);
	
	buff[0] = '\n';
	// copy server message in the buffer

	// and send that buffer to client
	write(sockfd, buff, sizeof(buff));

	// if msg contains "Exit" then server exit and chat ended.
	if (strncmp("exit", buff, 4) == 0) {
		printf("Server Exit...\n");
		close(sockfd);
	}
	return data;
}

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace solidBodyMotionFunctions
{
    defineTypeNameAndDebug(controlled6DoFMotion, 0);
    addToRunTimeSelectionTable
    (
        solidBodyMotionFunction,
        controlled6DoFMotion,
        dictionary
    );
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::solidBodyMotionFunctions::controlled6DoFMotion::controlled6DoFMotion
(
    const dictionary& SBMFCoeffs,
    const Time& runTime
)
:
    connfd(),solidBodyMotionFunction(SBMFCoeffs, runTime), socket_id(SBMFCoeffs_.get<scalar>("socket_id")), CofG(SBMFCoeffs_.get<vector>("centreOfGravity"))
{
	char socket_id_string[SOCKET_PATH_SIZE];
	char socket_name_[SOCKET_PATH_SIZE] = {0};
	snprintf(socket_id_string, SOCKET_PATH_SIZE, "%.0f", socket_id);
	strcat(socket_name_,"./exchange");
	strcat(socket_name_,socket_id_string);
	strcat(socket_name_,".sock");
	Pout << "socket name (constructor): " << socket_name_ << endl;
	
	
	read(SBMFCoeffs);

	Info << "Constructor" << endl;
	
	if (Pstream::master())
    {
		
		struct sockaddr_un servaddr;
		//struct sockaddr_in servaddr, cli;
		
		//unlink socket, if not terminated
		unlink(socket_name_);

		// socket create and verification
		connfd = socket(AF_UNIX, SOCK_STREAM, 0);
		if (connfd == -1) {
			printf("socket creation failed...\n");
			FatalErrorInFunction
		 	<< exit(FatalError);
		}
		else
			printf("Socket successfully created..\n");
		bzero(&servaddr, sizeof(servaddr));

		// assign IP, PORT
		servaddr.sun_family = AF_UNIX;
		strncpy(servaddr.sun_path, socket_name_, sizeof(servaddr.sun_path) - 1);

		// Binding newly created socket to given IP and verification
		if ((bind(connfd, (const struct sockaddr *) &servaddr, sizeof(struct sockaddr_un))) != 0) {
			printf("socket bind failed...\n");
			FatalErrorInFunction
		 	<< exit(FatalError);
		}
		else
			printf("Socket successfully bound...\n");

		// Now server is ready to listen and verification
		if ((listen(connfd, 5)) != 0) {
			printf("Listen failed...\n");
			FatalErrorInFunction
		 	<< exit(FatalError);
		}
		else
			printf("Server listening..\n");

		// Accept the data packet from client and verification
		connfd = accept(connfd, NULL, NULL);
		if (connfd < 0) {
			printf("server acccept failed...\n");
			FatalErrorInFunction
		 	<< exit(FatalError);
		}
		else
			printf("server acccept the client...\n");
	}
}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

Foam::septernion
Foam::solidBodyMotionFunctions::controlled6DoFMotion::transformation() const
{
    scalar t = time_.value();
    
    string data;
    string master_data;
	if (Pstream::master())
    {
    	std::cout << "Ich bin der Master proc (Member Functions) ..." << endl;
    	master_data = func(connfd);
	}
	
	Pstream::scatter(master_data);
	data = master_data;
	Pout<< "data: "<< data << endl;
	
	if(not Pstream::master())
	{
		Pout<< "Data slave: "<< data << endl;
	}
	else
	{
		Pout<< "Data master: "<< data << endl;
	}
	
	std::istringstream ss(data);
	std::string token;

	vector displacement;
	vector rotation;
	int i = 0;
	while(std::getline(ss, token, ';')) {
		i += 1;
		switch(i){
			case 1: displacement.x() = stof(token);
				break;
			case 2:	displacement.y() = stof(token);
				break;
			case 3:	displacement.z() = stof(token);
				break;
			case 4:	rotation.x() = stof(token);
				break;
			case 5:	rotation.y() = stof(token);
				break;
			case 6:	rotation.z() = stof(token);
				break;
		}
	}
	Info << "\nDisplacement: " << displacement;
	Info << "\nRotation: " << rotation;
    Info << "\nCofG_: " << CofG;
    
    scalar times = t;
    
    Info << "\ntimes: " << times;
    Info << "\nt: " << t;
    

    vector x(1,1,1);	//displacement
    vector y(2,2,2);	//rotation
    Vector2D<vector> TRV(displacement,rotation);
    Info << "\nTRV: " << TRV;
    
    quaternion R(quaternion::XYZ, TRV[1]);
    septernion TR(septernion(-CofG + -TRV[0])*R*septernion(CofG));
    

    DebugInFunction << "Time = " << t << " transformation: " << TR << endl;

    return TR;
}


bool Foam::solidBodyMotionFunctions::controlled6DoFMotion::read
(
    const dictionary& SBMFCoeffs
)
{
    solidBodyMotionFunction::read(SBMFCoeffs);

    return true;
}


// ************************************************************************* //
