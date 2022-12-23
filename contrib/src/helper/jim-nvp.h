/* SPDX-License-Identifier: BSD-2-Clause-Views */

/* Jim - A small embeddable Tcl interpreter
 *
 * Copyright 2005 Salvatore Sanfilippo <antirez@invece.org>
 * Copyright 2005 Clemens Hintze <c.hintze@gmx.net>
 * Copyright 2005 patthoyts - Pat Thoyts <patthoyts@users.sf.net>
 * Copyright 2008 oharboe - Øyvind Harboe - oyvind.harboe@zylin.com
 * Copyright 2008 Andrew Lunn <andrew@lunn.ch>
 * Copyright 2008 Duane Ellis <openocd@duaneellis.com>
 * Copyright 2008 Uwe Klein <uklein@klein-messgeraete.de>
 * Copyright 2008 Steve Bennett <steveb@workware.net.au>
 * Copyright 2009 Nico Coesel <ncoesel@dealogic.nl>
 * Copyright 2009 Zachary T Welch zw@superlucidity.net
 * Copyright 2009 David Brownell
 * Copyright (c) 2005-2011 Jim Tcl Project. All rights reserved.
 */

#ifndef OPENOCD_HELPER_JIM_NVP_H
#define OPENOCD_HELPER_JIM_NVP_H

#include <jim.h>

/** Name Value Pairs, aka: NVP
 *   -  Given a string - return the associated int.
 *   -  Given a number - return the associated string.
 *   .
 *
 * Very useful when the number is not a simple index into an array of
 * known string, or there may be multiple strings (aliases) that mean then same
 * thing.
 *
 * An NVP Table is terminated with ".name = NULL".
 *
 * During the 'name2value' operation, if no matching string is found
 * the pointer to the terminal element (with p->name == NULL) is returned.
 *
 * Example:
 * \code
 *      const struct jim_nvp yn[] = {
 *          { "yes", 1 },
 *          { "no" , 0 },
 *          { "yep", 1 },
 *          { "nope", 0 },
 *          { NULL, -1 },
 *      };
 *
 *  struct jim_nvp *result
 *  e = jim_nvp_name2value(interp, yn, "y", &result);
 *         returns &yn[0];
 *  e = jim_nvp_name2value(interp, yn, "n", &result);
 *         returns &yn[1];
 *  e = jim_nvp_name2value(interp, yn, "Blah", &result);
 *         returns &yn[4];
 * \endcode
 *
 * During the number2name operation, the first matching value is returned.
 */
struct jim_nvp {
	const char *name;
	int value;
};

int jim_get_nvp(Jim_Interp *interp,
		Jim_Obj *objptr,
		const struct jim_nvp *nvp_table,
		const struct jim_nvp **result);

/* Name Value Pairs Operations */
struct jim_nvp *jim_nvp_name2value_simple(const struct jim_nvp *nvp_table, const char *name);
struct jim_nvp *jim_nvp_name2value_nocase_simple(const struct jim_nvp *nvp_table, const char *name);
struct jim_nvp *jim_nvp_value2name_simple(const struct jim_nvp *nvp_table, int v);

int jim_nvp_name2value(Jim_Interp *interp,
		const struct jim_nvp *nvp_table,
		const char *name,
		struct jim_nvp **result);
int jim_nvp_name2value_nocase(Jim_Interp *interp,
		const struct jim_nvp *nvp_table,
		const char *name,
		struct jim_nvp **result);
int jim_nvp_value2name(Jim_Interp *interp, const struct jim_nvp *nvp_table, int value, struct jim_nvp **result);

int jim_nvp_name2value_obj(Jim_Interp *interp,
		const struct jim_nvp *nvp_table,
		Jim_Obj *name_obj,
		struct jim_nvp **result);
int jim_nvp_name2value_obj_nocase(Jim_Interp *interp,
		const struct jim_nvp *nvp_table,
		Jim_Obj *name_obj,
		struct jim_nvp **result);
int jim_nvp_value2name_obj(Jim_Interp *interp,
		const struct jim_nvp *nvp_table,
		Jim_Obj *value_obj,
		struct jim_nvp **result);

/** prints a nice 'unknown' parameter error message to the 'result' */
void jim_set_result_nvp_unknown(Jim_Interp *interp,
		Jim_Obj *param_name,
		Jim_Obj *param_value,
		const struct jim_nvp *nvp_table);

/** Debug: convert argc/argv into a printable string for printf() debug
 *
 * \param interp - the interpreter
 * \param argc   - arg count
 * \param argv   - the objects
 *
 * \returns string pointer holding the text.
 *
 * Note, next call to this function will free the old (last) string.
 *
 * For example might want do this:
 * \code
 *     fp = fopen("some.file.log", "a");
 *     fprintf(fp, "PARAMS are: %s\n", Jim_DebugArgvString(interp, argc, argv));
 *     fclose(fp);
 * \endcode
 */
const char *jim_debug_argv_string(Jim_Interp *interp, int argc, Jim_Obj *const *argv);


/** A TCL -ish GetOpt like code.
 *
 * Some TCL objects have various "configuration" values.
 * For example - in Tcl/Tk the "buttons" have many options.
 *
 * Useful when dealing with command options.
 * that may come in any order...
 *
 * Does not support "-foo = 123" type options.
 * Only supports tcl type options, like "-foo 123"
 */

struct jim_getopt_info {
	Jim_Interp *interp;
	int argc;
	Jim_Obj *const *argv;
	int isconfigure;		/* non-zero if configure */
};

/** GetOpt - how to.
 *
 * Example (short and incomplete):
 * \code
 *   struct jim_getopt_info goi;
 *
 *   jim_getopt_setup(&goi, interp, argc, argv);
 *
 *   while (goi.argc) {
 *         e = jim_getopt_nvp(&goi, nvp_options, &n);
 *         if (e != JIM_OK) {
 *               jim_getopt_nvp_unknown(&goi, nvp_options, 0);
 *               return e;
 *         }
 *
 *         switch (n->value) {
 *         case ALIVE:
 *             printf("Option ALIVE specified\n");
 *             break;
 *         case FIRST:
 *             if (goi.argc < 1) {
 *                     .. not enough args error ..
 *             }
 *             jim_getopt_string(&goi, &cp, NULL);
 *             printf("FIRSTNAME: %s\n", cp);
 *         case AGE:
 *             jim_getopt_wide(&goi, &w);
 *             printf("AGE: %d\n", (int)(w));
 *             break;
 *         case POLITICS:
 *             e = jim_getopt_nvp(&goi, nvp_politics, &n);
 *             if (e != JIM_OK) {
 *                 jim_getopt_nvp_unknown(&goi, nvp_politics, 1);
 *                 return e;
 *             }
 *         }
 *  }
 *
 * \endcode
 *
 */

/** Setup GETOPT
 *
 * \param goi    - get opt info to be initialized
 * \param interp - jim interp
 * \param argc   - argc count.
 * \param argv   - argv (will be copied)
 *
 * \code
 *     struct jim_getopt_info  goi;
 *
 *     Jim_GetOptSetup(&goi, interp, argc, argv);
 * \endcode
 */

int jim_getopt_setup(struct jim_getopt_info *goi,
		Jim_Interp *interp,
		int argc,
		Jim_Obj *const *argv);


/** Debug - Dump parameters to stderr
 * \param goi - current parameters
 */
void jim_getopt_debug(struct jim_getopt_info *goi);

/** Remove argv[0] from the list.
 *
 * \param goi - get opt info
 * \param puthere - where param is put
 *
 */
int jim_getopt_obj(struct jim_getopt_info *goi, Jim_Obj **puthere);

/** Remove argv[0] as string.
 *
 * \param goi     - get opt info
 * \param puthere - where param is put
 * \param len     - return its length
 */
int jim_getopt_string(struct jim_getopt_info *goi, const char **puthere, int *len);

/** Remove argv[0] as double.
 *
 * \param goi     - get opt info
 * \param puthere - where param is put.
 *
 */
int jim_getopt_double(struct jim_getopt_info *goi, double *puthere);

/** Remove argv[0] as wide.
 *
 * \param goi     - get opt info
 * \param puthere - where param is put.
 */
int jim_getopt_wide(struct jim_getopt_info *goi, jim_wide *puthere);

/** Remove argv[0] as NVP.
 *
 * \param goi     - get opt info
 * \param lookup  - nvp lookup table
 * \param puthere - where param is put.
 *
 */
int jim_getopt_nvp(struct jim_getopt_info *goi, const struct jim_nvp *lookup, struct jim_nvp **puthere);

/** Create an appropriate error message for an NVP.
 *
 * \param goi - options info
 * \param lookup - the NVP table that was used.
 * \param hadprefix - 0 or 1 if the option had a prefix.
 *
 * This function will set the "interp->result" to a human readable
 * error message listing the available options.
 *
 * This function assumes the previous option argv[-1] is the unknown string.
 *
 * If this option had some prefix, then pass "hadprefix = 1" else pass "hadprefix = 0"
 *
 * Example:
 * \code
 *
 *  while (goi.argc) {
 *     // Get the next option
 *     e = jim_getopt_nvp(&goi, cmd_options, &n);
 *     if (e != JIM_OK) {
 *          // option was not recognized
 *          // pass 'hadprefix = 0' because there is no prefix
 *          jim_getopt_nvp_unknown(&goi, cmd_options, 0);
 *          return e;
 *     }
 *
 *     switch (n->value) {
 *     case OPT_SEX:
 *          // handle:  --sex male | female | lots | needmore
 *          e = jim_getopt_nvp(&goi, &nvp_sex, &n);
 *          if (e != JIM_OK) {
 *               jim_getopt_nvp_unknown(&ogi, nvp_sex, 1);
 *               return e;
 *          }
 *          printf("Code: (%d) is %s\n", n->value, n->name);
 *          break;
 *     case ...:
 *          [snip]
 *     }
 * }
 * \endcode
 *
 */
void jim_getopt_nvp_unknown(struct jim_getopt_info *goi, const struct jim_nvp *lookup, int hadprefix);


/** Remove argv[0] as Enum
 *
 * \param goi     - get opt info
 * \param lookup  - lookup table.
 * \param puthere - where param is put.
 *
 */
int jim_getopt_enum(struct jim_getopt_info *goi, const char *const *lookup, int *puthere);

#endif /* OPENOCD_HELPER_JIM_NVP_H */
