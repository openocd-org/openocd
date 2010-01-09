/***************************************************************************
 *   Copyright (C) 2007,2008,2009 Øyvind Harboe                            *
 *   oyvind.harboe@zylin.com                                               *
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

/* some bits were copied from ahttpd which is under eCos license and
 * copyright to FSF
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "telnet_server.h"
#include <target/target.h>

#include <microhttpd.h>
#include <pthread.h>
#include <signal.h>

#define PAGE_NOT_FOUND "<html><head><title > File not found</title></head><body > File not found</body></html>"

static pthread_mutex_t mutex;

void openocd_sleep_prelude(void)
{
	pthread_mutex_unlock(&mutex);
}

void openocd_sleep_postlude(void)
{
	pthread_mutex_lock(&mutex);
}



int loadFile(const char *name, void **data, size_t *len);

static const char *appendf(const char *prev, const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	char *string = alloc_vprintf(format, ap);
	va_end(ap);
	char *string2 = NULL;

	if (string != NULL)
	{
		string2 = alloc_printf("%s%s", (prev == NULL) ? "" : prev, string);
	}

	if (prev != NULL)
	{
		free((void *)prev);
	}

	if (string == NULL)
		free(string);

	return string2;
}

static const char *httpd_exec_cgi_tcl_error(Jim_Interp *interp)
{
	int len, i;

	const char *t = NULL;
	t = appendf(t, "<html><body>\n");

	t = appendf(t, "Runtime error, file \"%s\", line %d:<br>",
			interp->errorFileName, interp->errorLine);
	t = appendf(t, "    %s < br>", Jim_GetString(interp->result, NULL));
	Jim_ListLength(interp, interp->stackTrace, &len);
	for (i = 0; i < len; i += 3)
	{
		Jim_Obj *objPtr;
		const char *proc, *file, *line;

		Jim_ListIndex(interp, interp->stackTrace, i, &objPtr, JIM_NONE);
		proc = Jim_GetString(objPtr, NULL);
		Jim_ListIndex(interp, interp->stackTrace, i + 1, &objPtr, JIM_NONE);
		file = Jim_GetString(objPtr, NULL);
		Jim_ListIndex(interp, interp->stackTrace, i + 2, &objPtr, JIM_NONE);
		line = Jim_GetString(objPtr, NULL);
		t = appendf(t, "In procedure '%s' called at file \"%s\", line %s < br>",
				proc, file, line);
	}
	t = appendf(t, "</html></body>\n");

	return t;
}

static int httpd_Jim_Command_writeform(Jim_Interp *interp, int argc,
		Jim_Obj * const *argv)
{
	if (argc != 3)
	{
		Jim_WrongNumArgs(interp, 1, argv, "method ?CMD_ARGV ...?");
		return JIM_ERR;
	}
	char *name = (char*) Jim_GetString(argv[1], NULL);
	char *file = (char*) Jim_GetString(argv[2], NULL);

	// Find length
	const char *data;
	int actual;
	int retcode;
	const char *script = alloc_printf(
			"set dummy_val $httppostdata(%s); set dummy_val",
			name);

	retcode = Jim_Eval_Named(interp, script, __FILE__, __LINE__);
	free((void *) script);
	if (retcode != JIM_OK)
		return retcode;

	data = Jim_GetString(Jim_GetResult(interp), &actual);

	FILE *f = fopen(file, "wb");
	if (NULL == f)
	{
		Jim_SetResultString(interp, "Could not create file", -1);
		return JIM_ERR;
	}

	int result = fwrite(data, 1, actual, f);
	fclose(f);

	if (result != actual)
	{
		Jim_SetResultString(interp, "Could not write to file", -1);
		return JIM_ERR;
	}
	return JIM_OK;
}


int
httpd_Jim_Command_formfetch(Jim_Interp *interp,
                                   int argc,
                                   Jim_Obj *const *argv)
{
	if (argc != 2)
	{
		Jim_WrongNumArgs(interp, 1, argv, "method ?CMD_ARGV ...?");
		return JIM_ERR;
	}

	char *name = (char*)Jim_GetString(argv[1], NULL);
	const char *script = alloc_printf(
		"set dummy_val $httppostdata(%s); set dummy_val",
		name);
	int retcode = Jim_Eval_Named(interp, script, __FILE__, __LINE__);

	free((void *) script);
	if (retcode != JIM_OK)
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
	else
		Jim_SetResult(interp, Jim_GetResult(interp));

	return JIM_OK;
}

struct httpd_request
{
	int post;
	Jim_Interp *interp;
	struct MHD_PostProcessor *postprocessor;

	//Jim_Obj *dict;

	int complete; /* did we receive the entire post ? */

};

static void request_completed(void *cls, struct MHD_Connection *connection,
		void **con_cls, enum MHD_RequestTerminationCode toe)
{
	struct httpd_request *r = (struct httpd_request*) *con_cls;

	if (NULL == r)
		return;

	if (r->postprocessor)
	{
		openocd_sleep_postlude();
		MHD_destroy_post_processor(r->postprocessor);
		openocd_sleep_prelude();
	}

	free(r);
	*con_cls = NULL;
}

/* append to said key in dictionary */
static void append_key(Jim_Interp *interp,
		struct httpd_request *r, const char *key,
		const char *data, size_t off, size_t size)
{
	Jim_Obj *keyObj = Jim_NewStringObj(interp, key, -1);
	Jim_IncrRefCount(keyObj);
	Jim_Obj *value = NULL;

	Jim_Obj *dict = Jim_GetVariableStr(interp, "httppostdata", 0);

	if (dict != NULL)
	{
		if (Jim_DictKey(interp, dict, keyObj, &value, 0) != JIM_OK)
		{
			 value = NULL;
		}
		else
		{
			Jim_IncrRefCount(value);
		}
	}

	if (value == NULL)
	{
		value = Jim_NewStringObj(interp, "", -1);
		Jim_IncrRefCount(value);

	}

	/* create a new object we append to and insert into this location */
	Jim_Obj *newObj = Jim_NewStringObj(interp, "", -1);
	Jim_IncrRefCount(newObj);
	Jim_AppendObj(interp, newObj, value);
	Jim_AppendString(interp, newObj, data, size);
	/* uhh... use name here of dictionary */
	dict = Jim_NewStringObj(interp, "httppostdata", -1);
	Jim_IncrRefCount(dict);
	Jim_SetDictKeysVector(interp, dict, &keyObj, 1, newObj);
	Jim_DecrRefCount(interp, dict);
	Jim_DecrRefCount(interp, value);
	Jim_DecrRefCount(interp, newObj);
	Jim_DecrRefCount(interp, keyObj);
}

/* append data to each key */
static int iterate_post(void *con_cls, enum MHD_ValueKind kind,
		const char *key, const char *filename, const char *content_type,
		const char *transfer_encoding, const char *data, uint64_t off,
		size_t size)
{
	struct httpd_request *r = (struct httpd_request*) con_cls;

	append_key(r->interp, r, key, data, off, size);

	return MHD_YES;
}

static int record_arg(void *cls, enum MHD_ValueKind kind, const char *key,
		const char *value)
{
	struct httpd_request *r = (struct httpd_request*) cls;
	append_key(r->interp, r, key, value, 0, strlen(value));
	return MHD_YES;
}


static int handle_request(Jim_Interp *interp,
		struct MHD_Connection * connection, const char * url)
{
	struct MHD_Response * response;

	int ret;
	const char *suffix;
	suffix = strrchr(url, '.');
	if ((suffix != NULL) && (strcmp(suffix, ".tcl") == 0))
	{
		printf("Run tcl %s\n", url);

		int retcode;

		const char *script = alloc_printf(
				"global httpdata; source {%s}; set httpdata", url);
		retcode = Jim_Eval_Named(interp, script, __FILE__, __LINE__);
		free((void *) script);

		if (retcode != JIM_OK)
		{
			printf("Tcl failed\n");
			const char *t = httpd_exec_cgi_tcl_error(interp);
			if (t == NULL)
				return MHD_NO;

			response = MHD_create_response_from_data(strlen(t), (void *) t,
					MHD_YES, MHD_NO);
			ret = MHD_queue_response(connection,
					MHD_HTTP_INTERNAL_SERVER_ERROR, response);
			MHD_destroy_response(response);
			return ret;
		}
		else
		{
			LOG_DEBUG("Tcl OK");
			/* FIX!!! how to handle mime types??? */
			const char *result;
			int reslen;
			result = Jim_GetString(Jim_GetResult(interp), &reslen);

			response = MHD_create_response_from_data(reslen, (void *) result,
					MHD_NO, MHD_YES);
			ret = MHD_queue_response(connection,
					MHD_HTTP_INTERNAL_SERVER_ERROR, response);
			MHD_destroy_response(response);
			return ret;
		}
	}
	else
	{
		void *data;
		size_t len;

		int retval = loadFile(url, &data, &len);
		if (retval != ERROR_OK)
		{
			printf("Did not find %s\n", url);

			response = MHD_create_response_from_data(strlen(PAGE_NOT_FOUND),
					(void *) PAGE_NOT_FOUND, MHD_NO, MHD_NO);
			ret = MHD_queue_response(connection, MHD_HTTP_NOT_FOUND, response);
			MHD_destroy_response(response);
			return ret;
		}

		LOG_DEBUG("Serving %s length=%zu", url, len);
		/* serve file directly */
		response = MHD_create_response_from_data(len, data, MHD_YES, MHD_NO);
		/* Should we expose mimetype via tcl here or just let the browser
		   guess?
		MHD_add_response_header(response, "Content-Type", "image/png");
		*/

		ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
		MHD_destroy_response(response);

		//free(data);
		return ret;
	}
}

static int ahc_echo_inner(void * cls, struct MHD_Connection * connection,
		const char * url, const char * method, const char * version,
		const char * upload_data, size_t * upload_data_size, void ** ptr)
{
	Jim_Interp *interp = (Jim_Interp *)cls;
	int post = 0;

	if (0 == strcmp(method, "POST"))
	{
		post = 1;
	}
	else if (0 == strcmp(method, "GET"))
	{
	}
	else
	{
		return MHD_NO; /* unexpected method */
	}

	struct httpd_request *r;
	if (*ptr == NULL)
	{
		/* The first time only the headers are valid,
		 do not respond in the first round... */

		*ptr = malloc(sizeof(struct httpd_request));
		if (*ptr == NULL)
			return MHD_NO;
		memset(*ptr, 0, sizeof(struct httpd_request));

		r = (struct httpd_request *) *ptr;
		r->interp = interp;
		r->post = post;
		Jim_SetVariableStr(interp, "httppostdata", Jim_NewDictObj(interp, NULL, 0));

		/* fill in url query strings in dictionary */
		MHD_get_connection_values(connection, MHD_GET_ARGUMENT_KIND,
				record_arg, r);

		if (r->post)
		{
			r->postprocessor = MHD_create_post_processor(connection, 2048
					* 1024, &iterate_post, r);
		}

		return MHD_YES;
	}

	r = (struct httpd_request *) *ptr;

	if (r->post)
	{
		/* consume post data */
		if (*upload_data_size)
		{
			MHD_post_process(r->postprocessor, upload_data, *upload_data_size);
			*upload_data_size = 0;
			return MHD_YES;
		}
		else
		{
		}
	} else
	{
	}

	/* hand over to request who will be using it. */
	//	r->dict = NULL;


	/* FIX!!!! we need more advanced handling of url's to avoid them
	 * being subverted to evil purposes
	 */

	const char *httpd_dir = PKGDATADIR "/httpd";

	if (*url=='/')
	{
		url++; /* skip '/' */
	}
	if (!*url)
		url="index.tcl";

	const char *file_name = alloc_printf("%s/%s", httpd_dir, url);
	int result = handle_request(interp, connection, file_name);
	free((void *)file_name);
	return result;
}


static int ahc_echo(void * cls, struct MHD_Connection * connection,
		const char * url, const char * method, const char * version,
		const char * upload_data, size_t * upload_data_size, void ** ptr)
{
	int result;

	openocd_sleep_postlude();

	result = ahc_echo_inner(cls, connection, url, method, version, upload_data, upload_data_size, ptr);

	openocd_sleep_prelude();

	return result;
}

static struct MHD_Daemon * d;

static const struct command_registration httpd_command_handlers[] = {
	{
		.name = "formfetch",
		.jim_handler = httpd_Jim_Command_formfetch,
		.mode = COMMAND_EXEC,
		.usage = "parameter_name",
		.help = "Reads a posted form value.",
	},
	{
		.name = "writeform",
		.jim_handler = httpd_Jim_Command_writeform,
		.mode = COMMAND_EXEC,
		.usage = "parameter_name filename",
		.help = "Writes a form value to a file.",
	},
	COMMAND_REGISTRATION_DONE
};

int httpd_start(struct command_context *cmd_ctx)
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&mutex, &attr);

	int port = 8888;
	LOG_USER("Launching httpd server on port %d", port);
	d = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, port, NULL, NULL,
			&ahc_echo, cmd_ctx->interp,
			MHD_OPTION_NOTIFY_COMPLETED, request_completed, NULL, /* Closure... what's that??? */
			MHD_OPTION_END);
	if (d == NULL)
		return ERROR_FAIL;

	return register_commands(cmd_ctx, NULL, httpd_command_handlers);
}

void httpd_stop(void)
{
	MHD_stop_daemon(d);
	pthread_mutex_destroy(&mutex);
}

