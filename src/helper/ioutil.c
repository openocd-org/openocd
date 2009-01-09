/***************************************************************************
 *   Copyright (C) 2007-2008 by Øyvind Harboe                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/* this file contains various functionality useful to standalone systems */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "types.h"
#include "configuration.h"
#include "target.h"

#include "command.h"

#include <time_support.h>
#include <sys/time.h>
#include <sys/types.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#if !BUILD_ECOSBOARD
#include <malloc.h>
#endif
#include <errno.h>


#include <fcntl.h>
#include <sys/stat.h>
#include <dirent.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>

#if !defined(__CYGWIN__)
#include <ifaddrs.h>
#endif

#include <unistd.h>
#include <stdio.h>

int handle_rm_command(struct command_context_s *cmd_ctx, char *cmd,
		char **args, int argc)
{
	if (argc != 1)
	{
		command_print(cmd_ctx, "rm <filename>");
		return ERROR_INVALID_ARGUMENTS;
	}

	if (unlink(args[0]) != 0)
	{
		command_print(cmd_ctx, "failed: %d", errno);
	}

	return ERROR_OK;
}


/* loads a file and returns a pointer to it in memory. The file contains
 * a 0 byte(sentinel) after len bytes - the length of the file. */
int loadFile(const char *fileName, void **data, int *len)
{
	FILE * pFile;
	pFile = fopen(fileName,"rb");
	if (pFile==NULL)
	{
		LOG_ERROR("Can't open %s\n", fileName);
		return ERROR_FAIL;
	}
	if (fseek(pFile, 0, SEEK_END)!=0)
	{
		LOG_ERROR("Can't open %s\n", fileName);
		fclose(pFile);
		return ERROR_FAIL;
	}
	*len=ftell(pFile);
	if (*len==-1)
	{
		LOG_ERROR("Can't open %s\n", fileName);
		fclose(pFile);
		return ERROR_FAIL;
	}

	if (fseek(pFile, 0, SEEK_SET)!=0)
	{
		LOG_ERROR("Can't open %s\n", fileName);
		fclose(pFile);
		return ERROR_FAIL;
	}
	*data=malloc(*len+1);
	if (*data==NULL)
	{
		LOG_ERROR("Can't open %s\n", fileName);
		fclose(pFile);
		return ERROR_FAIL;
	}

	if (fread(*data, 1, *len, pFile)!=*len)
	{
		fclose(pFile);
		free(*data);
		LOG_ERROR("Can't open %s\n", fileName);
		return ERROR_FAIL;
	}
	fclose(pFile);
	*(((char *)(*data))+*len)=0; /* sentinel */

	return ERROR_OK;



}



int handle_cat_command(struct command_context_s *cmd_ctx, char *cmd,
		char **args, int argc)
{
	if (argc != 1)
	{
		command_print(cmd_ctx, "cat <filename>");
		return ERROR_INVALID_ARGUMENTS;
	}

	// NOTE!!! we only have line printing capability so we print the entire file as a single line.
	void *data;
	int len;

	int retval = loadFile(args[0], &data, &len);
	if (retval == ERROR_OK)
	{
		command_print(cmd_ctx, "%s", data);
		free(data);
	}
	else
	{
		command_print(cmd_ctx, "%s not found %d", args[0], retval);
	}

	return ERROR_OK;
}
int handle_trunc_command(struct command_context_s *cmd_ctx, char *cmd,
		char **args, int argc)
{
	if (argc != 1)
	{
		command_print(cmd_ctx, "trunc <filename>");
		return ERROR_INVALID_ARGUMENTS;
	}

	FILE *config_file = NULL;
	config_file = fopen(args[0], "w");
	if (config_file != NULL)
		fclose(config_file);

	return ERROR_OK;
}


int handle_meminfo_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	static int prev = 0;
	struct mallinfo info;

	if (argc != 0)
	{
		command_print(cmd_ctx, "meminfo");
		return ERROR_INVALID_ARGUMENTS;
	}

	info = mallinfo();

	if (prev > 0)
	{
		command_print(cmd_ctx, "Diff:            %d", prev - info.fordblks);
	}
	prev = info.fordblks;

	command_print(cmd_ctx, "Available ram:   %d", info.fordblks );

	return ERROR_OK;
}


int handle_append_command(struct command_context_s *cmd_ctx, char *cmd,
		char **args, int argc)
{
	if (argc < 1)
	{
		command_print(cmd_ctx,
				"append <filename> [<string1>, [<string2>, ...]]");
		return ERROR_INVALID_ARGUMENTS;
	}

	FILE *config_file = NULL;
	config_file = fopen(args[0], "a");
	if (config_file != NULL)
	{
		int i;
		fseek(config_file, 0, SEEK_END);

		for (i = 1; i < argc; i++)
		{
			fwrite(args[i], strlen(args[i]), 1, config_file);
			if (i != argc - 1)
			{
				fwrite(" ", 1, 1, config_file);
			}
		}
		fwrite("\n", 1, 1, config_file);
		fclose(config_file);
	}

	return ERROR_OK;
}



int handle_cp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc != 2)
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	// NOTE!!! we only have line printing capability so we print the entire file as a single line.
	void *data;
	int len;

	int retval = loadFile(args[0], &data, &len);
	if (retval != ERROR_OK)
		return retval;

	FILE *f = fopen(args[1], "wb");
	if (f == NULL)
		retval = ERROR_INVALID_ARGUMENTS;

	int pos = 0;
	for (;;)
	{
		int chunk = len - pos;
		static const int maxChunk = 512 * 1024; // ~1/sec
		if (chunk > maxChunk)
		{
			chunk = maxChunk;
		}

		if ((retval==ERROR_OK)&&(fwrite(((char *)data)+pos, 1, chunk, f)!=chunk))
			retval = ERROR_INVALID_ARGUMENTS;

		if (retval != ERROR_OK)
		{
			break;
		}

		command_print(cmd_ctx, "%d", len - pos);

		pos += chunk;

		if (pos == len)
			break;
	}

	if (retval == ERROR_OK)
	{
		command_print(cmd_ctx, "Copied %s to %s", args[0], args[1]);
	} else
	{
		command_print(cmd_ctx, "Failed: %d", retval);
	}

	if (data != NULL)
		free(data);
	if (f != NULL)
		fclose(f);

	if (retval != ERROR_OK)
		unlink(args[1]);

	return retval;
}




#define SHOW_RESULT(a, b) LOG_ERROR(#a " failed %d\n", (int)b)

#define IOSIZE 512
void copyfile(char *name2, char *name1)
{

	int err;
	char buf[IOSIZE];
	int fd1, fd2;
	ssize_t done, wrote;

	fd1 = open(name1, O_WRONLY | O_CREAT);
	if (fd1 < 0)
		SHOW_RESULT( open, fd1 );

	fd2 = open(name2, O_RDONLY);
	if (fd2 < 0)
		SHOW_RESULT( open, fd2 );

	for (;;)
	{
		done = read(fd2, buf, IOSIZE );
		if (done < 0)
		{
			SHOW_RESULT( read, done );
			break;
		}

        if( done == 0 ) break;

		wrote = write(fd1, buf, done);
        if( wrote != done ) SHOW_RESULT( write, wrote );

        if( wrote != done ) break;
	}

	err = close(fd1);
    if( err < 0 ) SHOW_RESULT( close, err );

	err = close(fd2);
    if( err < 0 ) SHOW_RESULT( close, err );

}

/* utility fn to copy a directory */
void copydir(char *name, char *destdir)
{
	int err;
	DIR *dirp;

	dirp = opendir(destdir);
	if (dirp==NULL)
	{
		mkdir(destdir, 0777);
	} else
	{
		err = closedir(dirp);
	}

	dirp = opendir(name);
    if( dirp == NULL ) SHOW_RESULT( opendir, -1 );

	for (;;)
	{
		struct dirent *entry = readdir(dirp);

		if (entry == NULL)
			break;

		if (strcmp(entry->d_name, ".") == 0)
			continue;
		if (strcmp(entry->d_name, "..") == 0)
			continue;

		int isDir = 0;
		struct stat buf;
		char fullPath[PATH_MAX];
		strncpy(fullPath, name, PATH_MAX);
		strcat(fullPath, "/");
		strncat(fullPath, entry->d_name, PATH_MAX - strlen(fullPath));

		if (stat(fullPath, &buf) == -1)
		{
			LOG_ERROR("unable to read status from %s", fullPath);
			break;
		}
		isDir = S_ISDIR(buf.st_mode) != 0;

		if (isDir)
			continue;

		//        diag_printf("<INFO>: entry %14s",entry->d_name);
		char fullname[PATH_MAX];
		char fullname2[PATH_MAX];

		strcpy(fullname, name);
		strcat(fullname, "/");
		strcat(fullname, entry->d_name);

		strcpy(fullname2, destdir);
		strcat(fullname2, "/");
		strcat(fullname2, entry->d_name);
		//        diag_printf("from %s to %s\n", fullname, fullname2);
		copyfile(fullname, fullname2);

		//       diag_printf("\n");
	}

	err = closedir(dirp);
    if( err < 0 ) SHOW_RESULT( stat, err );
}




static int
zylinjtag_Jim_Command_rm(Jim_Interp *interp,
                                   int argc,
		Jim_Obj * const *argv)
{
	int del;
	if (argc != 2)
	{
		Jim_WrongNumArgs(interp, 1, argv, "rm ?dirorfile?");
		return JIM_ERR;
	}

	del = 0;
	if (unlink(Jim_GetString(argv[1], NULL)) == 0)
		del = 1;
	if (rmdir(Jim_GetString(argv[1], NULL)) == 0)
		del = 1;

	return del ? JIM_OK : JIM_ERR;
}


static int
zylinjtag_Jim_Command_ls(Jim_Interp *interp,
                                   int argc,
		Jim_Obj * const *argv)
{
	if (argc != 2)
	{
		Jim_WrongNumArgs(interp, 1, argv, "ls ?dir?");
		return JIM_ERR;
	}

	char *name = (char*) Jim_GetString(argv[1], NULL);

	DIR *dirp = NULL;
	dirp = opendir(name);
	if (dirp == NULL)
	{
		return JIM_ERR;
	}
	Jim_Obj *objPtr = Jim_NewListObj(interp, NULL, 0);

	for (;;)
	{
		struct dirent *entry = NULL;
		entry = readdir(dirp);
		if (entry == NULL)
			break;

		if ((strcmp(".", entry->d_name)==0)||(strcmp("..", entry->d_name)==0))
			continue;

        Jim_ListAppendElement(interp, objPtr, Jim_NewStringObj(interp, entry->d_name, strlen(entry->d_name)));
	}
	closedir(dirp);

	Jim_SetResult(interp, objPtr);

	return JIM_OK;
}

int handle_peek_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc != 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	volatile int *address=(volatile int *)strtoul(args[0], NULL, 0);
	int value=*address;
	command_print(cmd_ctx, "0x%x : 0x%x", address, value);
	return ERROR_OK;
}

int handle_poke_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc != 2)
	{
		return ERROR_INVALID_ARGUMENTS;
	}
	volatile int *address=(volatile int *)strtoul(args[0], NULL, 0);
	int value=strtoul(args[1], NULL, 0);
	*address=value;
	return ERROR_OK;
}

static int
zylinjtag_Jim_Command_peek(Jim_Interp *interp,
                                   int argc,
		Jim_Obj * const *argv)
{
	if (argc != 2)
	{
		Jim_WrongNumArgs(interp, 1, argv, "peek ?address?");
		return JIM_ERR;
	}

	long address;
	if (Jim_GetLong(interp, argv[1], &address) != JIM_OK)
		return JIM_ERR;

	int value = *((volatile int *) address);

	Jim_SetResult(interp, Jim_NewIntObj(interp, value));

	return JIM_OK;
}

static int
zylinjtag_Jim_Command_poke(Jim_Interp *interp,
                                   int argc,
		Jim_Obj * const *argv)
{
	if (argc != 3)
	{
		Jim_WrongNumArgs(interp, 1, argv, "poke ?address? ?value?");
		return JIM_ERR;
	}

	long address;
	if (Jim_GetLong(interp, argv[1], &address) != JIM_OK)
		return JIM_ERR;
	long value;
	if (Jim_GetLong(interp, argv[2], &value) != JIM_OK)
		return JIM_ERR;

	*((volatile int *) address) = value;

	return JIM_OK;
}


/* not so pretty code to fish out ip number*/
static int zylinjtag_Jim_Command_ip(Jim_Interp *interp, int argc,
		Jim_Obj * const *argv)
{
#if !defined(__CYGWIN__)
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);

	struct ifaddrs *ifa = NULL, *ifp = NULL;

	if (getifaddrs(&ifp) < 0)
	{
		return JIM_ERR;
	}

	for (ifa = ifp; ifa; ifa = ifa->ifa_next)
	{
		char ip[200];
		socklen_t salen;

		if (ifa->ifa_addr->sa_family == AF_INET)
			salen = sizeof(struct sockaddr_in);
		else if (ifa->ifa_addr->sa_family == AF_INET6)
			salen = sizeof(struct sockaddr_in6);
		else
			continue;

		if (getnameinfo(ifa->ifa_addr, salen, ip, sizeof(ip), NULL, 0,
				NI_NUMERICHOST) < 0)
		{
			continue;
		}

		Jim_AppendString(interp, tclOutput, ip, strlen(ip));
		break;

	}

	freeifaddrs(ifp);
#else
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "fixme!!!", 0);
	LOG_ERROR("NOT IMPLEMENTED!!!");
#endif
	Jim_SetResult(interp, tclOutput);

	return JIM_OK;
}

/* not so pretty code to fish out eth0 mac address */
static int zylinjtag_Jim_Command_mac(Jim_Interp *interp, int argc,
		Jim_Obj * const *argv)
{


	struct ifreq *ifr, *ifend;
	struct ifreq ifreq;
	struct ifconf ifc;
	struct ifreq ifs[5];
	int SockFD;

	SockFD = socket(AF_INET, SOCK_DGRAM, 0);
	if (SockFD < 0)
	{
		return JIM_ERR;
	}

	ifc.ifc_len = sizeof(ifs);
	ifc.ifc_req = ifs;
	if (ioctl(SockFD, SIOCGIFCONF, &ifc) < 0)
	{
		close(SockFD);
		return JIM_ERR;
	}

	ifend = ifs + (ifc.ifc_len / sizeof(struct ifreq));
	for (ifr = ifc.ifc_req; ifr < ifend; ifr++)
	{
		//if (ifr->ifr_addr.sa_family == AF_INET)
		{
			if (strcmp("eth0", ifr->ifr_name)!=0)
				continue;
			strncpy(ifreq.ifr_name, ifr->ifr_name, sizeof(ifreq.ifr_name));
			if (ioctl(SockFD, SIOCGIFHWADDR, &ifreq) < 0)
			{
				close(SockFD);
				return JIM_ERR;
			}

			close(SockFD);


			Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);

			char buffer[256];
			sprintf(buffer, "%02x-%02x-%02x-%02x-%02x-%02x",
					ifreq.ifr_hwaddr.sa_data[0]&0xff,
					ifreq.ifr_hwaddr.sa_data[1]&0xff,
					ifreq.ifr_hwaddr.sa_data[2]&0xff,
					ifreq.ifr_hwaddr.sa_data[3]&0xff,
					ifreq.ifr_hwaddr.sa_data[4]&0xff,
					ifreq.ifr_hwaddr.sa_data[5]&0xff);

			Jim_AppendString(interp, tclOutput, buffer, strlen(buffer));

			Jim_SetResult(interp, tclOutput);

			return JIM_OK;
		}
	}
	close(SockFD);

	return JIM_ERR;

}



int ioutil_init(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "rm", handle_rm_command, COMMAND_ANY,
			"remove file");

	register_command(cmd_ctx, NULL, "cat", handle_cat_command, COMMAND_ANY,
			"display file content");

	register_command(cmd_ctx, NULL, "trunc", handle_trunc_command, COMMAND_ANY,
			"truncate a file to 0 size");

	register_command(cmd_ctx, NULL, "cp", handle_cp_command,
					 COMMAND_ANY, "copy a file <from> <to>");

	register_command(cmd_ctx, NULL, "append_file", handle_append_command,
			COMMAND_ANY, "append a variable number of strings to a file");

	register_command(cmd_ctx, NULL, "meminfo", handle_meminfo_command,
			COMMAND_ANY, "display available ram memory");

    Jim_CreateCommand(interp, "rm", zylinjtag_Jim_Command_rm, NULL, NULL);

    Jim_CreateCommand(interp, "peek", zylinjtag_Jim_Command_peek, NULL, NULL);
    Jim_CreateCommand(interp, "poke", zylinjtag_Jim_Command_poke, NULL, NULL);
    Jim_CreateCommand(interp, "ls", zylinjtag_Jim_Command_ls, NULL, NULL);

	Jim_CreateCommand(interp, "mac", zylinjtag_Jim_Command_mac,
			NULL, NULL);

	Jim_CreateCommand(interp, "ip", zylinjtag_Jim_Command_ip,
			NULL, NULL);

    return ERROR_OK;
}


