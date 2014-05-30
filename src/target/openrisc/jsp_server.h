#ifndef _JSP_SERVER_H_
#define _JSP_SERVER_H_

#include "or1k_tap.h"
#include "or1k.h"
#include "or1k_du.h"

struct jsp_service {
	char *banner;
	struct or1k_jtag *jtag_info;
	struct connection *connection;
};

int jsp_init(struct or1k_jtag *jtag_info, char *banner);
int jsp_register_commands(struct command_context *cmd_ctx);

#endif	/* _JSP_SERVER_H_ */
