/* Jim - A small embeddable Tcl interpreter
 *
 * Copyright 2005 Salvatore Sanfilippo <antirez@invece.org>
 * Copyright 2005 Clemens Hintze <c.hintze@gmx.net>
 * Copyright 2005 patthoyts - Pat Thoyts <patthoyts@users.sf.net> 
 * Copyright 2008 oharboe - Øyvind Harboe - oyvind.harboe@zylin.com
 * Copyright 2008 Andrew Lunn <andrew@lunn.ch>
 * Copyright 2008 Duane Ellis <openocd@duaneellis.com>
 * Copyright 2008 Uwe Klein <uklein@klein-messgeraete.de>
 * 
 * The FreeBSD license
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE JIM TCL PROJECT ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * JIM TCL PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation
 * are those of the authors and should not be interpreted as representing
 * official policies, either expressed or implied, of the Jim Tcl Project.
 *
 *--- Inline Header File Documentation --- 
 *    [By Duane Ellis, openocd@duaneellis.com, 8/18/8]
 *
 * Belief is "Jim" would greatly benifit if Jim Internals where
 * documented in some way - form whatever, and perhaps - the package:
 * 'doxygen' is the correct approach to do that.
 *
 *   Details, see: http://www.stack.nl/~dimitri/doxygen/
 *
 * To that end please follow these guide lines:
 *
 *    (A) Document the PUBLIC api in the .H file.
 *
 *    (B) Document JIM Internals, in the .C file.
 *
 *    (C) Remember JIM is embedded in other packages, to that end do
 *    not assume that your way of documenting is the right way, Jim's
 *    public documentation should be agnostic, such that it is some
 *    what agreeable with the "package" that is embedding JIM inside
 *    of it's own doxygen documentation.
 *
 *    (D) Use minimal Doxygen tags.
 *
 * This will be an "ongoing work in progress" for some time.
 **/

#ifndef __JIM__H
#define __JIM__H

#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>
#include <limits.h>
#include <stdio.h>  /* for the FILE typedef definition */
#include <stdlib.h> /* In order to export the Jim_Free() macro */
#include <stdarg.h> /* In order to get type va_list */

/* -----------------------------------------------------------------------------
* Some /very/ old compiler maybe do not know how to
* handle 'const'. They even do not know, how to ignore
* it. For those compiler it may be better to compile with
* define JIM_NO_CONST activated
* ---------------------------------------------------------------------------*/

#ifdef JIM_NO_CONST
#  define const
#endif

/* -----------------------------------------------------------------------------
 * System configuration
 * For most modern systems, you can leave the default.
 * For embedded systems some change may be required.
 * ---------------------------------------------------------------------------*/

#define HAVE_LONG_LONG

/* -----------------------------------------------------------------------------
 * Compiler specific fixes.
 * ---------------------------------------------------------------------------*/

/* MSC has _stricmp instead of strcasecmp */
#ifdef _MSC_VER
#  define strcasecmp _stricmp
#endif /* _MSC_VER */

/* Long Long type and related issues */
#ifdef HAVE_LONG_LONG
#  ifdef _MSC_VER /* MSC compiler */
#    define jim_wide _int64
#    ifndef LLONG_MAX
#      define LLONG_MAX    9223372036854775807I64
#    endif
#    ifndef LLONG_MIN
#      define LLONG_MIN    (-LLONG_MAX - 1I64)
#    endif
#    define JIM_WIDE_MIN LLONG_MIN
#    define JIM_WIDE_MAX LLONG_MAX
#  else /* Other compilers (mainly GCC) */
#    define jim_wide long long
#    ifndef LLONG_MAX
#      define LLONG_MAX    9223372036854775807LL
#    endif
#    ifndef LLONG_MIN
#      define LLONG_MIN    (-LLONG_MAX - 1LL)
#    endif
#    define JIM_WIDE_MIN LLONG_MIN
#    define JIM_WIDE_MAX LLONG_MAX
#  endif
#else
#  define jim_wide long
#  define JIM_WIDE_MIN LONG_MIN
#  define JIM_WIDE_MAX LONG_MAX
#endif

/* -----------------------------------------------------------------------------
 * LIBC specific fixes
 * ---------------------------------------------------------------------------*/

#ifdef HAVE_LONG_LONG
# if defined(_MSC_VER) || defined(__MSVCRT__)
#    define JIM_WIDE_MODIFIER "I64d"
# else
#    define JIM_WIDE_MODIFIER "lld"
# endif
#else
#    define JIM_WIDE_MODIFIER "ld"
#endif

/* -----------------------------------------------------------------------------
 * Exported defines
 * ---------------------------------------------------------------------------*/

/* Jim version numbering: every version of jim is marked with a
 * successive integer number. This is version 0. The first
 * stable version will be 1, then 2, 3, and so on. */
#define JIM_VERSION 51

#define JIM_OK 0
#define JIM_ERR 1
#define JIM_RETURN 2
#define JIM_BREAK 3
#define JIM_CONTINUE 4
#define JIM_EVAL 5
#define JIM_EXIT 6
#define JIM_ERR_ADDSTACK 7
#define JIM_MAX_NESTING_DEPTH 10000 /* default max nesting depth */

/* Some function get an integer argument with flags to change
 * the behaviour. */
#define JIM_NONE 0    /* no flags set */
#define JIM_ERRMSG 1    /* set an error message in the interpreter. */

/* Flags for Jim_SubstObj() */
#define JIM_SUBST_NOVAR 1 /* don't perform variables substitutions */
#define JIM_SUBST_NOCMD 2 /* don't perform command substitutions */
#define JIM_SUBST_NOESC 4 /* don't perform escapes substitutions */

/* Unused arguments generate annoying warnings... */
#define JIM_NOTUSED(V) ((void) V)

/* Flags used by API calls getting a 'nocase' argument. */
#define JIM_CASESENS    0   /* case sensitive */
#define JIM_NOCASE      1   /* no case */

/* Filesystem related */
#define JIM_PATH_LEN 1024

/* Newline, some embedded system may need -DJIM_CRLF */
#ifdef JIM_CRLF
#define JIM_NL "\r\n"
#else
#define JIM_NL "\n"
#endif

#if defined(__WIN32__) || defined(_WIN32)
#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)
#else
#define DLLEXPORT
#define DLLIMPORT
#endif

/* -----------------------------------------------------------------------------
 * Stack
 * ---------------------------------------------------------------------------*/

typedef struct Jim_Stack {
    int len;
    int maxlen;
    void **vector;
} Jim_Stack;

/* -----------------------------------------------------------------------------
 * Hash table
 * ---------------------------------------------------------------------------*/

typedef struct Jim_HashEntry {
    const void *key;
    void *val;
    struct Jim_HashEntry *next;
} Jim_HashEntry;

typedef struct Jim_HashTableType {
    unsigned int (*hashFunction)(const void *key);
    const void *(*keyDup)(void *privdata, const void *key);
    void *(*valDup)(void *privdata, const void *obj);
    int (*keyCompare)(void *privdata, const void *key1, const void *key2);
    void (*keyDestructor)(void *privdata, const void *key);
    void (*valDestructor)(void *privdata, void *obj);
} Jim_HashTableType;

typedef struct Jim_HashTable {
    Jim_HashEntry **table;
    Jim_HashTableType *type;
    unsigned int size;
    unsigned int sizemask;
    unsigned int used;
    unsigned int collisions;
    void *privdata;
} Jim_HashTable;

typedef struct Jim_HashTableIterator {
    Jim_HashTable *ht;
    int index;
    Jim_HashEntry *entry, *nextEntry;
} Jim_HashTableIterator;

/* This is the initial size of every hash table */
#define JIM_HT_INITIAL_SIZE     16

/* ------------------------------- Macros ------------------------------------*/
#define Jim_FreeEntryVal(ht, entry) \
    if ((ht)->type->valDestructor) \
        (ht)->type->valDestructor((ht)->privdata, (entry)->val)

#define Jim_SetHashVal(ht, entry, _val_) do { \
    if ((ht)->type->valDup) \
        entry->val = (ht)->type->valDup((ht)->privdata, _val_); \
    else \
        entry->val = (_val_); \
} while(0)

#define Jim_FreeEntryKey(ht, entry) \
    if ((ht)->type->keyDestructor) \
        (ht)->type->keyDestructor((ht)->privdata, (entry)->key)

#define Jim_SetHashKey(ht, entry, _key_) do { \
    if ((ht)->type->keyDup) \
        entry->key = (ht)->type->keyDup((ht)->privdata, _key_); \
    else \
        entry->key = (_key_); \
} while(0)

#define Jim_CompareHashKeys(ht, key1, key2) \
    (((ht)->type->keyCompare) ? \
        (ht)->type->keyCompare((ht)->privdata, key1, key2) : \
        (key1) == (key2))

#define Jim_HashKey(ht, key) (ht)->type->hashFunction(key)

#define Jim_GetHashEntryKey(he) ((he)->key)
#define Jim_GetHashEntryVal(he) ((he)->val)
#define Jim_GetHashTableCollisions(ht) ((ht)->collisions)
#define Jim_GetHashTableSize(ht) ((ht)->size)
#define Jim_GetHashTableUsed(ht) ((ht)->used)

/* -----------------------------------------------------------------------------
 * Jim_Obj structure
 * ---------------------------------------------------------------------------*/

/* -----------------------------------------------------------------------------
 * Jim object. This is mostly the same as Tcl_Obj itself,
 * with the addition of the 'prev' and 'next' pointers.
 * In Jim all the objects are stored into a linked list for GC purposes,
 * so that it's possible to access every object living in a given interpreter
 * sequentially. When an object is freed, it's moved into a different
 * linked list, used as object pool.
 *
 * The refcount of a freed object is always -1.
 * ---------------------------------------------------------------------------*/
typedef struct Jim_Obj {
    int refCount; /* reference count */
    char *bytes; /* string representation buffer. NULL = no string repr. */
    int length; /* number of bytes in 'bytes', not including the numterm. */
    struct Jim_ObjType *typePtr; /* object type. */
    /* Internal representation union */
    union {
        /* integer number type */
        jim_wide wideValue;
        /* hashed object type value */
        int hashValue;
        /* index type */
        int indexValue;
        /* return code type */
        int returnCode;
        /* double number type */
        double doubleValue;
        /* Generic pointer */
        void *ptr;
        /* Generic two pointers value */
        struct {
            void *ptr1;
            void *ptr2;
        } twoPtrValue;
        /* Variable object */
        struct {
            unsigned jim_wide callFrameId;
            struct Jim_Var *varPtr;
        } varValue;
        /* Command object */
        struct {
            unsigned jim_wide procEpoch;
            struct Jim_Cmd *cmdPtr;
        } cmdValue;
        /* List object */
        struct {
            struct Jim_Obj **ele;    /* Elements vector */
            int len;        /* Length */
            int maxLen;        /* Allocated 'ele' length */
        } listValue;
        /* String type */
        struct {
            int maxLength;
        } strValue;
        /* Reference type */
        struct {
            jim_wide id;
            struct Jim_Reference *refPtr;
        } refValue;
        /* Source type */
        struct {
            const char *fileName;
            int lineNumber;
        } sourceValue;
        /* Dict substitution type */
        struct {
            struct Jim_Obj *varNameObjPtr;
            struct Jim_Obj *indexObjPtr;
        } dictSubstValue;
        /* tagged binary type */
        struct {
            unsigned char *data;
            size_t         len;
        } binaryValue;
    } internalRep;
    /* This are 8 or 16 bytes more for every object
     * but this is required for efficient garbage collection
     * of Jim references. */
    struct Jim_Obj *prevObjPtr; /* pointer to the prev object. */
    struct Jim_Obj *nextObjPtr; /* pointer to the next object. */
} Jim_Obj;

/* Jim_Obj related macros */
#define Jim_IncrRefCount(objPtr) \
    ++(objPtr)->refCount
#define Jim_DecrRefCount(interp, objPtr) \
    if (--(objPtr)->refCount <= 0) Jim_FreeObj(interp, objPtr)
#define Jim_IsShared(objPtr) \
    ((objPtr)->refCount > 1)

/* This macro is used when we allocate a new object using
 * Jim_New...Obj(), but for some error we need to destroy it.
 * Instead to use Jim_IncrRefCount() + Jim_DecrRefCount() we
 * can just call Jim_FreeNewObj. To call Jim_Free directly
 * seems too raw, the object handling may change and we want
 * that Jim_FreeNewObj() can be called only against objects
 * that are belived to have refcount == 0. */
#define Jim_FreeNewObj Jim_FreeObj

/* Free the internal representation of the object. */
#define Jim_FreeIntRep(i,o) \
    if ((o)->typePtr && (o)->typePtr->freeIntRepProc) \
        (o)->typePtr->freeIntRepProc(i, o)

/* Get the internal representation pointer */
#define Jim_GetIntRepPtr(o) (o)->internalRep.ptr

/* Set the internal representation pointer */
#define Jim_SetIntRepPtr(o, p) \
    (o)->internalRep.ptr = (p)

/* The object type structure.
 * There are four methods.
 *
 * - FreeIntRep is used to free the internal representation of the object.
 *   Can be NULL if there is nothing to free.
 * - DupIntRep is used to duplicate the internal representation of the object.
 *   If NULL, when an object is duplicated, the internalRep union is
 *   directly copied from an object to another.
 *   Note that it's up to the caller to free the old internal repr of the
 *   object before to call the Dup method.
 * - UpdateString is used to create the string from the internal repr.
 * - setFromAny is used to convert the current object into one of this type.
 */

struct Jim_Interp;

typedef void (Jim_FreeInternalRepProc)(struct Jim_Interp *interp,
        struct Jim_Obj *objPtr);
typedef void (Jim_DupInternalRepProc)(struct Jim_Interp *interp,
        struct Jim_Obj *srcPtr, Jim_Obj *dupPtr);
typedef void (Jim_UpdateStringProc)(struct Jim_Obj *objPtr);
    
typedef struct Jim_ObjType {
    const char *name; /* The name of the type. */
    Jim_FreeInternalRepProc *freeIntRepProc;
    Jim_DupInternalRepProc *dupIntRepProc;
    Jim_UpdateStringProc *updateStringProc;
    int flags;
} Jim_ObjType;

/* Jim_ObjType flags */
#define JIM_TYPE_NONE 0        /* No flags */
#define JIM_TYPE_REFERENCES 1    /* The object may contain referneces. */

/* Starting from 1 << 20 flags are reserved for private uses of
 * different calls. This way the same 'flags' argument may be used
 * to pass both global flags and private flags. */
#define JIM_PRIV_FLAG_SHIFT 20

/* -----------------------------------------------------------------------------
 * Call frame, vars, commands structures
 * ---------------------------------------------------------------------------*/

/* Call frame */
typedef struct Jim_CallFrame {
    unsigned jim_wide id; /* Call Frame ID. Used for caching. */
    struct Jim_HashTable vars; /* Where local vars are stored */
    struct Jim_HashTable *staticVars; /* pointer to procedure static vars */
    struct Jim_CallFrame *parentCallFrame;
    Jim_Obj *const *argv; /* object vector of the current procedure call. */
    int argc; /* number of args of the current procedure call. */
    Jim_Obj *procArgsObjPtr; /* arglist object of the running procedure */
    Jim_Obj *procBodyObjPtr; /* body object of the running procedure */
    struct Jim_CallFrame *nextFramePtr;
} Jim_CallFrame;

/* The var structure. It just holds the pointer of the referenced
 * object. If linkFramePtr is not NULL the variable is a link
 * to a variable of name store on objPtr living on the given callframe
 * (this happens when the [global] or [upvar] command is used).
 * The interp in order to always know how to free the Jim_Obj associated
 * with a given variable because In Jim objects memory managment is
 * bound to interpreters. */
typedef struct Jim_Var {
    Jim_Obj *objPtr;
    struct Jim_CallFrame *linkFramePtr;
} Jim_Var;
    
/* The cmd structure. */
typedef int (*Jim_CmdProc)(struct Jim_Interp *interp, int argc,
    Jim_Obj *const *argv);
typedef void (*Jim_DelCmdProc)(struct Jim_Interp *interp, void *privData);

/* A command is implemented in C if funcPtr is != NULL, otherwise
 * it's a Tcl procedure with the arglist and body represented by the
 * two objects referenced by arglistObjPtr and bodyoObjPtr. */
typedef struct Jim_Cmd {
    Jim_CmdProc cmdProc; /* Not-NULL for a C command. */
    void *privData; /* Only used for C commands. */
    Jim_DelCmdProc delProc; /* Called when the command is deleted if != NULL */
    Jim_Obj *argListObjPtr;
    Jim_Obj *bodyObjPtr;
    Jim_HashTable *staticVars; /* Static vars hash table. NULL if no statics. */
    int arityMin; /* Min number of arguments. */
    int arityMax; /* Max number of arguments. */
} Jim_Cmd;

/* Pseudo Random Number Generator State structure */
typedef struct Jim_PrngState {
    unsigned char sbox[256];
    unsigned int i, j;
} Jim_PrngState;

/* -----------------------------------------------------------------------------
 * Jim interpreter structure.
 * Fields similar to the real Tcl interpreter structure have the same names.
 * ---------------------------------------------------------------------------*/
typedef struct Jim_Interp {
    Jim_Obj *result; /* object returned by the last command called. */
    int errorLine; /* Error line where an error occurred. */
    const char *errorFileName; /* Error file where an error occurred. */
    int numLevels; /* Number of current nested calls. */
    int maxNestingDepth; /* Used for infinite loop detection. */
    int returnCode; /* Completion code to return on JIM_RETURN. */
    int exitCode; /* Code to return to the OS on JIM_EXIT. */
    Jim_CallFrame *framePtr; /* Pointer to the current call frame */
    Jim_CallFrame *topFramePtr; /* toplevel/global frame pointer. */
    struct Jim_HashTable commands; /* Commands hash table */
    unsigned jim_wide procEpoch; /* Incremented every time the result
                of procedures names lookup caching
                may no longer be valid. */
    unsigned jim_wide callFrameEpoch; /* Incremented every time a new
                callframe is created. This id is used for the
                'ID' field contained in the Jim_CallFrame
                structure. */
    Jim_Obj *liveList; /* Linked list of all the live objects. */
    Jim_Obj *freeList; /* Linked list of all the unused objects. */
    const char *scriptFileName; /* File name of the script currently in execution. */
    Jim_Obj *emptyObj; /* Shared empty string object. */
    unsigned jim_wide referenceNextId; /* Next id for reference. */
    struct Jim_HashTable references; /* References hash table. */
    jim_wide lastCollectId; /* reference max Id of the last GC
                execution. It's set to -1 while the collection
                is running as sentinel to avoid to recursive
                calls via the [collect] command inside
                finalizers. */
    time_t lastCollectTime; /* unix time of the last GC execution */
    struct Jim_HashTable sharedStrings; /* Shared Strings hash table */
    Jim_Obj *stackTrace; /* Stack trace object. */
    Jim_Obj *unknown; /* Unknown command cache */
    int unknown_called; /* The unknown command has been invoked */
    int errorFlag; /* Set if an error occurred during execution. */
    int evalRetcodeLevel; /* Level where the last return with code JIM_EVAL
                             happened. */
    void *cmdPrivData; /* Used to pass the private data pointer to
                  a command. It is set to what the user specified
                  via Jim_CreateCommand(). */

    struct Jim_HashTable stub; /* Stub hash table to export API */
    /* Jim_GetApi() function pointer, used to bootstrap the STUB table */
    int (*getApiFuncPtr)(struct Jim_Interp *, const char *, void *);
    struct Jim_CallFrame *freeFramesList; /* list of CallFrame structures. */
    struct Jim_HashTable assocData; /* per-interp storage for use by packages */
    Jim_PrngState *prngState; /* per interpreter Random Number Gen. state. */
    struct Jim_HashTable packages; /* Provided packages hash table */
    void *cookie_stdin; /* input file pointer, 'stdin' by default */
    void *cookie_stdout; /* output file pointer, 'stdout' by default */
    void *cookie_stderr; /* errors file pointer, 'stderr' by default */
    size_t (*cb_fwrite  )( const void *ptr, size_t size, size_t n, void *cookie );
	size_t (*cb_fread   )( void *ptr, size_t size, size_t n, void *cookie );
	int    (*cb_vfprintf)( void *cookie, const char *fmt, va_list ap );
	int    (*cb_fflush  )( void *cookie );
	char  *(*cb_fgets   )( char *s, int size, void *cookie );
} Jim_Interp;

/* Currently provided as macro that performs the increment.
 * At some point may be a real function doing more work.
 * The proc epoch is used in order to know when a command lookup
 * cached can no longer considered valid. */
#define Jim_InterpIncrProcEpoch(i) (i)->procEpoch++
#define Jim_SetResultString(i,s,l) Jim_SetResult(i, Jim_NewStringObj(i,s,l))
#define Jim_SetResultInt(i,intval) Jim_SetResult(i, Jim_NewIntObj(i,intval))
#define Jim_SetEmptyResult(i) Jim_SetResult(i, (i)->emptyObj)
#define Jim_GetResult(i) ((i)->result)
#define Jim_CmdPrivData(i) ((i)->cmdPrivData)

/* Note that 'o' is expanded only one time inside this macro,
 * so it's safe to use side effects. */
#define Jim_SetResult(i,o) do {     \
    Jim_Obj *_resultObjPtr_ = (o);    \
    Jim_IncrRefCount(_resultObjPtr_); \
    Jim_DecrRefCount(i,(i)->result);  \
    (i)->result = _resultObjPtr_;     \
} while(0)

/* Reference structure. The interpreter pointer is held within privdata member in HashTable */
#define JIM_REFERENCE_TAGLEN 7 /* The tag is fixed-length, because the reference
                                  string representation must be fixed length. */
typedef struct Jim_Reference {
    Jim_Obj *objPtr;
    Jim_Obj *finalizerCmdNamePtr;
    char tag[JIM_REFERENCE_TAGLEN+1];
} Jim_Reference;

/** Name Value Pairs, aka: NVP
 *   -  Given a string - return the associated int.
 *   -  Given a number - return the associated string.
 *   .
 *
 * Very useful when the number is not a simple index into an array of
 * known string, or there may be multiple strings (aliases) that mean then same
 * thing.
 *
 * An NVP Table is terminated with ".name=NULL".
 *
 * During the 'name2value' operation, if no matching string is found
 * the pointer to the terminal element (with p->name==NULL) is returned.
 *
 * Example:
 * \code
 *      const Jim_Nvp yn[] = {
 *          { "yes", 1 },
 *          { "no" , 0 },
 *          { "yep", 1 },
 *          { "nope", 0 },
 *          { NULL, -1 },
 *      };
 *
 *  Jim_Nvp *result
 *  e = Jim_Nvp_name2value( interp, yn, "y", &result ); 
 *         returns &yn[0];
 *  e = Jim_Nvp_name2value( interp, yn, "n", &result );
 *         returns &yn[1];
 *  e = Jim_Nvp_name2value( interp, yn, "Blah", &result );
 *         returns &yn[4];
 * \endcode
 *
 * During the number2name operation, the first matching value is returned.
 */
typedef struct {
	const char *name;
	int         value;
} Jim_Nvp;
    

/* -----------------------------------------------------------------------------
 * Exported API prototypes.
 * ---------------------------------------------------------------------------*/

/* Macros that are common for extensions and core. */
#define Jim_NewEmptyStringObj(i) Jim_NewStringObj(i, "", 0)

/* The core includes real prototypes, extensions instead
 * include a global function pointer for every function exported.
 * Once the extension calls Jim_InitExtension(), the global
 * functon pointers are set to the value of the STUB table
 * contained in the Jim_Interp structure.
 *
 * This makes Jim able to load extensions even if it is statically
 * linked itself, and to load extensions compiled with different
 * versions of Jim (as long as the API is still compatible.) */

/* Macros are common for core and extensions */
#define Jim_FreeHashTableIterator(iter) Jim_Free(iter)

#ifdef DOXYGEN
#define JIM_STATIC 
#define JIM_API( X )  X
#else
#ifndef __JIM_CORE__
# if defined JIM_EXTENSION || defined JIM_EMBEDDED
#  define JIM_API(x) (*x)
#  define JIM_STATIC
# else
#  define JIM_API(x) (*x)
#  define JIM_STATIC extern
# endif
#else
# define JIM_API(x) x
# if defined(BUILD_Jim)
#   define JIM_STATIC DLLEXPORT
# else
#   define JIM_STATIC static
# endif
#endif /* __JIM_CORE__ */
#endif /* DOXYGEN */

/** Set the result - printf() style */
JIM_STATIC int JIM_API( Jim_SetResult_sprintf )( Jim_Interp *p, const char *fmt, ... );

/* Memory allocation */
JIM_STATIC void * JIM_API(Jim_Alloc) (int size);
JIM_STATIC void JIM_API(Jim_Free) (void *ptr);
JIM_STATIC char * JIM_API(Jim_StrDup) (const char *s);

/* evaluation */
JIM_STATIC int JIM_API(Jim_Eval)(Jim_Interp *interp, const char *script);
/* in C code, you can do this and get better error messages */
/*   Jim_Eval_Named( interp, "some tcl commands", __FILE__, __LINE__ ); */
JIM_STATIC int JIM_API(Jim_Eval_Named)(Jim_Interp *interp, const char *script,const char *filename, int lineno);
JIM_STATIC int JIM_API(Jim_EvalGlobal)(Jim_Interp *interp, const char *script);
JIM_STATIC int JIM_API(Jim_EvalFile)(Jim_Interp *interp, const char *filename);
JIM_STATIC int JIM_API(Jim_EvalObj) (Jim_Interp *interp, Jim_Obj *scriptObjPtr);
JIM_STATIC int JIM_API(Jim_EvalObjBackground) (Jim_Interp *interp,
        Jim_Obj *scriptObjPtr);
JIM_STATIC int JIM_API(Jim_EvalObjVector) (Jim_Interp *interp, int objc,
        Jim_Obj *const *objv);
JIM_STATIC int JIM_API(Jim_SubstObj) (Jim_Interp *interp, Jim_Obj *substObjPtr,
        Jim_Obj **resObjPtrPtr, int flags);

/* stack */
JIM_STATIC void JIM_API(Jim_InitStack)(Jim_Stack *stack);
JIM_STATIC void JIM_API(Jim_FreeStack)(Jim_Stack *stack);
JIM_STATIC int JIM_API(Jim_StackLen)(Jim_Stack *stack);
JIM_STATIC void JIM_API(Jim_StackPush)(Jim_Stack *stack, void *element);
JIM_STATIC void * JIM_API(Jim_StackPop)(Jim_Stack *stack);
JIM_STATIC void * JIM_API(Jim_StackPeek)(Jim_Stack *stack);
JIM_STATIC void JIM_API(Jim_FreeStackElements)(Jim_Stack *stack, void (*freeFunc)(void *ptr));

/* hash table */
JIM_STATIC int JIM_API(Jim_InitHashTable) (Jim_HashTable *ht,
        Jim_HashTableType *type, void *privdata);
JIM_STATIC int JIM_API(Jim_ExpandHashTable) (Jim_HashTable *ht,
        unsigned int size);
JIM_STATIC int JIM_API(Jim_AddHashEntry) (Jim_HashTable *ht, const void *key,
        void *val);
JIM_STATIC int JIM_API(Jim_ReplaceHashEntry) (Jim_HashTable *ht,
        const void *key, void *val);
JIM_STATIC int JIM_API(Jim_DeleteHashEntry) (Jim_HashTable *ht,
        const void *key);
JIM_STATIC int JIM_API(Jim_FreeHashTable) (Jim_HashTable *ht);
JIM_STATIC Jim_HashEntry * JIM_API(Jim_FindHashEntry) (Jim_HashTable *ht,
        const void *key);
JIM_STATIC int JIM_API(Jim_ResizeHashTable) (Jim_HashTable *ht);
JIM_STATIC Jim_HashTableIterator *JIM_API(Jim_GetHashTableIterator)
        (Jim_HashTable *ht);
JIM_STATIC Jim_HashEntry * JIM_API(Jim_NextHashEntry)
        (Jim_HashTableIterator *iter);

/* objects */
JIM_STATIC Jim_Obj * JIM_API(Jim_NewObj) (Jim_Interp *interp);
JIM_STATIC void JIM_API(Jim_FreeObj) (Jim_Interp *interp, Jim_Obj *objPtr);
JIM_STATIC void JIM_API(Jim_InvalidateStringRep) (Jim_Obj *objPtr);
JIM_STATIC void JIM_API(Jim_InitStringRep) (Jim_Obj *objPtr, const char *bytes,
        int length);
JIM_STATIC Jim_Obj * JIM_API(Jim_DuplicateObj) (Jim_Interp *interp,
        Jim_Obj *objPtr);
JIM_STATIC const char * JIM_API(Jim_GetString)(Jim_Obj *objPtr,
        int *lenPtr);
JIM_STATIC int JIM_API(Jim_Length)(Jim_Obj *objPtr);

/* string object */
JIM_STATIC Jim_Obj * JIM_API(Jim_NewStringObj) (Jim_Interp *interp,
        const char *s, int len);
JIM_STATIC Jim_Obj * JIM_API(Jim_NewStringObjNoAlloc) (Jim_Interp *interp,
        char *s, int len);
JIM_STATIC void JIM_API(Jim_AppendString) (Jim_Interp *interp, Jim_Obj *objPtr,
        const char *str, int len);
JIM_STATIC void JIM_API(Jim_AppendString_sprintf) (Jim_Interp *interp, Jim_Obj *objPtr,
												   const char *fmt, ... );
JIM_STATIC void JIM_API(Jim_AppendObj) (Jim_Interp *interp, Jim_Obj *objPtr,
        Jim_Obj *appendObjPtr);
JIM_STATIC void JIM_API(Jim_AppendStrings) (Jim_Interp *interp,
        Jim_Obj *objPtr, ...);
JIM_STATIC int JIM_API(Jim_StringEqObj) (Jim_Obj *aObjPtr,
        Jim_Obj *bObjPtr, int nocase);
JIM_STATIC int JIM_API(Jim_StringMatchObj) (Jim_Obj *patternObjPtr,
        Jim_Obj *objPtr, int nocase);
JIM_STATIC Jim_Obj * JIM_API(Jim_StringRangeObj) (Jim_Interp *interp,
        Jim_Obj *strObjPtr, Jim_Obj *firstObjPtr,
        Jim_Obj *lastObjPtr);
JIM_STATIC Jim_Obj * JIM_API(Jim_FormatString) (Jim_Interp *interp,
        Jim_Obj *fmtObjPtr, int objc, Jim_Obj *const *objv);
JIM_STATIC Jim_Obj * JIM_API(Jim_ScanString) (Jim_Interp *interp, Jim_Obj *strObjPtr,
        Jim_Obj *fmtObjPtr, int flags);
JIM_STATIC int JIM_API(Jim_CompareStringImmediate) (Jim_Interp *interp,
        Jim_Obj *objPtr, const char *str);

/* reference object */
JIM_STATIC Jim_Obj * JIM_API(Jim_NewReference) (Jim_Interp *interp,
        Jim_Obj *objPtr, Jim_Obj *tagPtr, Jim_Obj *cmdNamePtr);
JIM_STATIC Jim_Reference * JIM_API(Jim_GetReference) (Jim_Interp *interp,
        Jim_Obj *objPtr);
JIM_STATIC int JIM_API(Jim_SetFinalizer) (Jim_Interp *interp, Jim_Obj *objPtr, Jim_Obj *cmdNamePtr);
JIM_STATIC int JIM_API(Jim_GetFinalizer) (Jim_Interp *interp, Jim_Obj *objPtr, Jim_Obj **cmdNamePtrPtr);

/* interpreter */
JIM_STATIC Jim_Interp * JIM_API(Jim_CreateInterp) (void);
JIM_STATIC void JIM_API(Jim_FreeInterp) (Jim_Interp *i);
JIM_STATIC int JIM_API(Jim_GetExitCode) (Jim_Interp *interp);
JIM_STATIC void * JIM_API(Jim_SetStdin) (Jim_Interp *interp, void *fp);
JIM_STATIC void * JIM_API(Jim_SetStdout) (Jim_Interp *interp, void *fp);
JIM_STATIC void * JIM_API(Jim_SetStderr) (Jim_Interp *interp, void *fp);

/* commands */
JIM_STATIC void JIM_API(Jim_RegisterCoreCommands) (Jim_Interp *interp);
JIM_STATIC int JIM_API(Jim_CreateCommand) (Jim_Interp *interp, 
        const char *cmdName, Jim_CmdProc cmdProc, void *privData,
         Jim_DelCmdProc delProc);
JIM_STATIC int JIM_API(Jim_CreateProcedure) (Jim_Interp *interp, 
        const char *cmdName, Jim_Obj *argListObjPtr, Jim_Obj *staticsListObjPtr,
        Jim_Obj *bodyObjPtr, int arityMin, int arityMax);
JIM_STATIC int JIM_API(Jim_DeleteCommand) (Jim_Interp *interp,
        const char *cmdName);
JIM_STATIC int JIM_API(Jim_RenameCommand) (Jim_Interp *interp, 
        const char *oldName, const char *newName);
JIM_STATIC Jim_Cmd * JIM_API(Jim_GetCommand) (Jim_Interp *interp,
        Jim_Obj *objPtr, int flags);
JIM_STATIC int JIM_API(Jim_SetVariable) (Jim_Interp *interp,
        Jim_Obj *nameObjPtr, Jim_Obj *valObjPtr);
JIM_STATIC int JIM_API(Jim_SetVariableStr) (Jim_Interp *interp,
        const char *name, Jim_Obj *objPtr);
JIM_STATIC int JIM_API(Jim_SetGlobalVariableStr) (Jim_Interp *interp,
        const char *name, Jim_Obj *objPtr);
JIM_STATIC int JIM_API(Jim_SetVariableStrWithStr) (Jim_Interp *interp,
        const char *name, const char *val);
JIM_STATIC int JIM_API(Jim_SetVariableLink) (Jim_Interp *interp,
        Jim_Obj *nameObjPtr, Jim_Obj *targetNameObjPtr,
        Jim_CallFrame *targetCallFrame);
JIM_STATIC Jim_Obj * JIM_API(Jim_GetVariable) (Jim_Interp *interp,
        Jim_Obj *nameObjPtr, int flags);
JIM_STATIC Jim_Obj * JIM_API(Jim_GetGlobalVariable) (Jim_Interp *interp,
        Jim_Obj *nameObjPtr, int flags);
JIM_STATIC Jim_Obj * JIM_API(Jim_GetVariableStr) (Jim_Interp *interp,
        const char *name, int flags);
JIM_STATIC Jim_Obj * JIM_API(Jim_GetGlobalVariableStr) (Jim_Interp *interp,
        const char *name, int flags);
JIM_STATIC int JIM_API(Jim_UnsetVariable) (Jim_Interp *interp,
        Jim_Obj *nameObjPtr, int flags);

/* call frame */
JIM_STATIC int JIM_API(Jim_GetCallFrameByLevel) (Jim_Interp *interp,
        Jim_Obj *levelObjPtr, Jim_CallFrame **framePtrPtr,
        int *newLevelPtr);

/* garbage collection */
JIM_STATIC int JIM_API(Jim_Collect) (Jim_Interp *interp);
JIM_STATIC void JIM_API(Jim_CollectIfNeeded) (Jim_Interp *interp);

/* index object */
JIM_STATIC int JIM_API(Jim_GetIndex) (Jim_Interp *interp, Jim_Obj *objPtr,
        int *indexPtr);

/* list object */
JIM_STATIC Jim_Obj * JIM_API(Jim_NewListObj) (Jim_Interp *interp,
        Jim_Obj *const *elements, int len);
JIM_STATIC void JIM_API(Jim_ListInsertElements) (Jim_Interp *interp,
        Jim_Obj *listPtr, int index, int objc, Jim_Obj *const *objVec);
JIM_STATIC void JIM_API(Jim_ListAppendElement) (Jim_Interp *interp,
        Jim_Obj *listPtr, Jim_Obj *objPtr);
JIM_STATIC void JIM_API(Jim_ListAppendList) (Jim_Interp *interp,
        Jim_Obj *listPtr, Jim_Obj *appendListPtr);
JIM_STATIC void JIM_API(Jim_ListLength) (Jim_Interp *interp, Jim_Obj *listPtr,
        int *intPtr);
JIM_STATIC int JIM_API(Jim_ListIndex) (Jim_Interp *interp, Jim_Obj *listPrt,
        int index, Jim_Obj **objPtrPtr, int seterr);
JIM_STATIC int JIM_API(Jim_SetListIndex) (Jim_Interp *interp,
        Jim_Obj *varNamePtr, Jim_Obj *const *indexv, int indexc,
        Jim_Obj *newObjPtr);
JIM_STATIC Jim_Obj * JIM_API(Jim_ConcatObj) (Jim_Interp *interp, int objc,
        Jim_Obj *const *objv);

/* dict object */
JIM_STATIC Jim_Obj * JIM_API(Jim_NewDictObj) (Jim_Interp *interp,
        Jim_Obj *const *elements, int len);
JIM_STATIC int JIM_API(Jim_DictKey) (Jim_Interp *interp, Jim_Obj *dictPtr,
        Jim_Obj *keyPtr, Jim_Obj **objPtrPtr, int flags);
JIM_STATIC int JIM_API(Jim_DictKeysVector) (Jim_Interp *interp,
        Jim_Obj *dictPtr, Jim_Obj *const *keyv, int keyc,
        Jim_Obj **objPtrPtr, int flags);
JIM_STATIC int JIM_API(Jim_SetDictKeysVector) (Jim_Interp *interp,
        Jim_Obj *varNamePtr, Jim_Obj *const *keyv, int keyc,
        Jim_Obj *newObjPtr);

/* return code object */
JIM_STATIC int JIM_API(Jim_GetReturnCode) (Jim_Interp *interp, Jim_Obj *objPtr,
        int *intPtr);

/* expression object */
JIM_STATIC int JIM_API(Jim_EvalExpression) (Jim_Interp *interp,
        Jim_Obj *exprObjPtr, Jim_Obj **exprResultPtrPtr);
JIM_STATIC int JIM_API(Jim_GetBoolFromExpr) (Jim_Interp *interp,
        Jim_Obj *exprObjPtr, int *boolPtr);

/* integer object */
JIM_STATIC int JIM_API(Jim_GetWide) (Jim_Interp *interp, Jim_Obj *objPtr,
        jim_wide *widePtr);
JIM_STATIC int JIM_API(Jim_GetLong) (Jim_Interp *interp, Jim_Obj *objPtr,
        long *longPtr);
JIM_STATIC void JIM_API(Jim_SetWide) (Jim_Interp *interp, Jim_Obj *objPtr,
        jim_wide wideValue);
#define Jim_NewWideObj  Jim_NewIntObj
JIM_STATIC Jim_Obj * JIM_API(Jim_NewIntObj) (Jim_Interp *interp,
        jim_wide wideValue);

/* double object */
JIM_STATIC int JIM_API(Jim_GetDouble)(Jim_Interp *interp, Jim_Obj *objPtr,
        double *doublePtr);
JIM_STATIC void JIM_API(Jim_SetDouble)(Jim_Interp *interp, Jim_Obj *objPtr,
        double doubleValue);
JIM_STATIC Jim_Obj * JIM_API(Jim_NewDoubleObj)(Jim_Interp *interp, double doubleValue);

/* shared strings */
JIM_STATIC const char * JIM_API(Jim_GetSharedString) (Jim_Interp *interp, 
        const char *str);
JIM_STATIC void JIM_API(Jim_ReleaseSharedString) (Jim_Interp *interp,
        const char *str);

/* commands utilities */
JIM_STATIC void JIM_API(Jim_WrongNumArgs) (Jim_Interp *interp, int argc,
        Jim_Obj *const *argv, const char *msg);
JIM_STATIC int JIM_API(Jim_GetEnum) (Jim_Interp *interp, Jim_Obj *objPtr,
        const char * const *tablePtr, int *indexPtr, const char *name, int flags);
JIM_STATIC int JIM_API(Jim_GetNvp) (Jim_Interp *interp, 
									Jim_Obj *objPtr,
									const Jim_Nvp *nvp_table, 
									const Jim_Nvp **result);
JIM_STATIC int JIM_API(Jim_ScriptIsComplete) (const char *s, int len,
        char *stateCharPtr);

/* package utilities */
typedef void (Jim_InterpDeleteProc)(Jim_Interp *interp, void *data);
JIM_STATIC void * JIM_API(Jim_GetAssocData)(Jim_Interp *interp, const char *key);
JIM_STATIC int JIM_API(Jim_SetAssocData)(Jim_Interp *interp, const char *key,
        Jim_InterpDeleteProc *delProc, void *data);
JIM_STATIC int JIM_API(Jim_DeleteAssocData)(Jim_Interp *interp, const char *key);

/* API import/export functions */
JIM_STATIC int JIM_API(Jim_GetApi) (Jim_Interp *interp, const char *funcname,
        void *targetPtrPtr);
JIM_STATIC int JIM_API(Jim_RegisterApi) (Jim_Interp *interp, 
        const char *funcname, void *funcptr);

/* Packages C API */
JIM_STATIC int JIM_API(Jim_PackageProvide) (Jim_Interp *interp,
        const char *name, const char *ver, int flags);
JIM_STATIC const char * JIM_API(Jim_PackageRequire) (Jim_Interp *interp,
        const char *name, const char *ver, int flags);

/* error messages */
JIM_STATIC void JIM_API(Jim_PrintErrorMessage) (Jim_Interp *interp);

/* interactive mode */
JIM_STATIC int JIM_API(Jim_InteractivePrompt) (Jim_Interp *interp);

/* Misc */
JIM_STATIC void JIM_API(Jim_Panic) (Jim_Interp *interp, const char *fmt, ...);

/* Jim's STDIO */
JIM_STATIC int     JIM_API( Jim_fprintf  )( Jim_Interp *interp, void *cookie, const char *fmt, ... );
JIM_STATIC int     JIM_API( Jim_vfprintf )( Jim_Interp *interp, void *cookie, const char *fmt, va_list ap );
JIM_STATIC size_t  JIM_API( Jim_fwrite   )( Jim_Interp *interp, const void *ptr, size_t size, size_t nmeb, void *cookie );
JIM_STATIC size_t  JIM_API( Jim_fread    )( Jim_Interp *interp, void *ptr, size_t size, size_t nmeb, void *cookie );
JIM_STATIC int     JIM_API( Jim_fflush   )( Jim_Interp *interp, void *cookie );
JIM_STATIC char *  JIM_API( Jim_fgets    )( Jim_Interp *interp, char *s, int size, void *cookie );

/* Name Value Pairs Operations */
JIM_STATIC Jim_Nvp *JIM_API(Jim_Nvp_name2value_simple)( const Jim_Nvp *nvp_table, const char *name );
JIM_STATIC Jim_Nvp *JIM_API(Jim_Nvp_name2value_nocase_simple)( const Jim_Nvp *nvp_table, const char *name );
JIM_STATIC Jim_Nvp *JIM_API(Jim_Nvp_value2name_simple)( const Jim_Nvp *nvp_table, int v );

JIM_STATIC int JIM_API(Jim_Nvp_name2value)( Jim_Interp *interp, const Jim_Nvp *nvp_table, const char *name, Jim_Nvp **result );
JIM_STATIC int JIM_API(Jim_Nvp_name2value_nocase)( Jim_Interp *interp, const Jim_Nvp *nvp_table, const char *name, Jim_Nvp **result);
JIM_STATIC int JIM_API(Jim_Nvp_value2name)( Jim_Interp *interp, const Jim_Nvp *nvp_table, int value, Jim_Nvp **result );

JIM_STATIC int JIM_API(Jim_Nvp_name2value_obj)( Jim_Interp *interp, const Jim_Nvp *nvp_table, Jim_Obj *name_obj, Jim_Nvp **result );
JIM_STATIC int JIM_API(Jim_Nvp_name2value_obj_nocase)( Jim_Interp *interp, const Jim_Nvp *nvp_table, Jim_Obj *name_obj, Jim_Nvp **result );
JIM_STATIC int JIM_API(Jim_Nvp_value2name_obj)( Jim_Interp *interp, const Jim_Nvp *nvp_table, Jim_Obj *value_obj, Jim_Nvp **result );

/** prints a nice 'unknown' parameter error message to the 'result' */
JIM_STATIC void JIM_API(Jim_SetResult_NvpUnknown)( Jim_Interp *interp, 
												   Jim_Obj *param_name,
												   Jim_Obj *param_value,
												   const Jim_Nvp *nvp_table );


/** Debug: convert argc/argv into a printable string for printf() debug
 * 
 * \param interp - the interpeter
 * \param argc   - arg count
 * \param argv   - the objects
 *
 * \returns string pointer holding the text.
 * 
 * Note, next call to this function will free the old (last) string.
 *
 * For example might want do this:
 * \code
 *     fp = fopen("some.file.log", "a" );
 *     fprintf( fp, "PARAMS are: %s\n", Jim_DebugArgvString( interp, argc, argv ) );
 *     fclose(fp);
 * \endcode
 */
JIM_STATIC const char *JIM_API( Jim_Debug_ArgvString )( Jim_Interp *interp, int argc, Jim_Obj *const *argv );


/** A TCL -ish GetOpt like code. 
 *
 * Some TCL objects have various "configuration" values.
 * For example - in Tcl/Tk the "buttons" have many options.
 * 
 * Usefull when dealing with command options.
 * that may come in any order...
 *
 * Does not support "-foo=123" type options.
 * Only supports tcl type options, like "-foo 123"
 */

typedef struct jim_getopt {
	Jim_Interp     *interp;
	int            argc; 
	Jim_Obj        * const * argv;
	int            isconfigure; /* non-zero if configure */
} Jim_GetOptInfo;

/** GetOpt - how to.
 *
 * Example (short and incomplete):
 * \code
 *   Jim_GetOptInfo goi; 
 *
 *   Jim_GetOpt_Setup( &goi, interp, argc, argv );
 *
 *   while( goi.argc ){
 *         e = Jim_GetOpt_Nvp( &goi, nvp_options, &n );
 *         if( e != JIM_OK ){
 *               Jim_GetOpt_NvpUnknown( &goi, nvp_options, 0 );
 *               return e;
 *         }
 *
 *         switch( n->value ){
 *         case ALIVE:
 *             printf("Option ALIVE specified\n");
 *             break;
 *         case FIRST:
 *             if( goi.argc < 1 ){
 *                     .. not enough args error ..
 *             }
 *             Jim_GetOpt_String( &goi, &cp, NULL );
 *             printf("FIRSTNAME: %s\n", cp );
 *         case AGE:
 *             Jim_GetOpt_Wide( &goi, &w );
 *             printf("AGE: %d\n", (int)(w) );
 *             break;
 *         case POLITICS:
 *             e = Jim_GetOpt_Nvp( &goi, nvp_politics, &n );
 *             if( e != JIM_OK ){
 *                 Jim_GetOpt_NvpUnknown( &goi, nvp_politics, 1 );
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
 *     Jim_GetOptInfo  goi;
 *   
 *     Jim_GetOptSetup( &goi, interp, argc, argv );
 * \endcode
 */

JIM_STATIC int JIM_API( Jim_GetOpt_Setup )( Jim_GetOptInfo *goi, 
											Jim_Interp *interp, 
											int argc, 
											Jim_Obj * const *  argv );


/** Debug - Dump parameters to stderr
 * \param goi - current parameters
 */
JIM_STATIC void JIM_API( Jim_GetOpt_Debug )( Jim_GetOptInfo *goi);



/** Remove argv[0] from the list.
 *
 * \param goi - get opt info
 * \param puthere - where param is put
 * 
 */
JIM_STATIC int JIM_API( Jim_GetOpt_Obj)( Jim_GetOptInfo *goi, Jim_Obj **puthere );

/** Remove argv[0] as string.
 *
 * \param goi     - get opt info
 * \param puthere - where param is put
 */
JIM_STATIC int JIM_API( Jim_GetOpt_String )( Jim_GetOptInfo *goi, char **puthere, int *len );

/** Remove argv[0] as double.
 *
 * \param goi     - get opt info
 * \param puthere - where param is put.
 *
 */
JIM_STATIC int JIM_API( Jim_GetOpt_Double )( Jim_GetOptInfo *goi, double *puthere );

/** Remove argv[0] as wide.
 *
 * \param goi     - get opt info
 * \param puthere - where param is put.
 */
JIM_STATIC int JIM_API( Jim_GetOpt_Wide )( Jim_GetOptInfo *goi, jim_wide *puthere );

/** Remove argv[0] as NVP.
 *
 * \param goi     - get opt info
 * \param lookup  - nvp lookup table
 * \param puthere - where param is put.
 *
 */
JIM_STATIC int JIM_API( Jim_GetOpt_Nvp)( Jim_GetOptInfo *goi, const Jim_Nvp *lookup, Jim_Nvp **puthere );

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
 * If this option had some prefix, then pass "hadprefix=1" else pass "hadprefix=0"
 *
 * Example:
 * \code
 *
 *  while( goi.argc ){
 *     // Get the next option 
 *     e = Jim_GetOpt_Nvp( &goi, cmd_options, &n );
 *     if( e != JIM_OK ){
 *          // option was not recognized
 *          // pass 'hadprefix=0' because there is no prefix
 *          Jim_GetOpt_NvpUnknown( &goi, cmd_options, 0 );
 *          return e;
 *     }
 *
 *     switch( n->value ){
 *     case OPT_SEX:
 *          // handle:  --sex male|female|lots|needmore
 *          e = Jim_GetOpt_Nvp( &goi, &nvp_sex, &n );
 *          if( e != JIM_OK ){
 *               Jim_GetOpt_NvpUnknown( &ogi, nvp_sex, 1 );
 *               return e;
 *          }
 *          printf("Code: (%d) is %s\n", n->value, n->name );
 *          break;
 *     case ...:
 *          [snip]
 *     }
 * }
 * \endcode
 *
 */
JIM_STATIC void JIM_API( Jim_GetOpt_NvpUnknown)( Jim_GetOptInfo *goi, const Jim_Nvp *lookup, int hadprefix );


/** Remove argv[0] as Enum
 *
 * \param goi     - get opt info
 * \param lookup  - lookup table.
 * \param puthere - where param is put.
 *
 */
JIM_STATIC int JIM_API( Jim_GetOpt_Enum)( Jim_GetOptInfo *goi, const char * const *  lookup, int *puthere );


#undef JIM_STATIC
#undef JIM_API

#ifndef __JIM_CORE__

#define JIM_GET_API(name) \
    Jim_GetApi(interp, "Jim_" #name, ((void *)&Jim_ ## name))

#if defined JIM_EXTENSION || defined JIM_EMBEDDED
/* This must be included "inline" inside the extension */
static void Jim_InitExtension(Jim_Interp *interp)
{
  Jim_GetApi = interp->getApiFuncPtr;

  JIM_GET_API(Alloc);
  JIM_GET_API(Free);
  JIM_GET_API(Eval);
  JIM_GET_API(Eval_Named);
  JIM_GET_API(EvalGlobal);
  JIM_GET_API(EvalFile);
  JIM_GET_API(EvalObj);
  JIM_GET_API(EvalObjBackground);
  JIM_GET_API(EvalObjVector);
  JIM_GET_API(InitHashTable);
  JIM_GET_API(ExpandHashTable);
  JIM_GET_API(AddHashEntry);
  JIM_GET_API(ReplaceHashEntry);
  JIM_GET_API(DeleteHashEntry);
  JIM_GET_API(FreeHashTable);
  JIM_GET_API(FindHashEntry);
  JIM_GET_API(ResizeHashTable);
  JIM_GET_API(GetHashTableIterator);
  JIM_GET_API(NextHashEntry);
  JIM_GET_API(NewObj);
  JIM_GET_API(FreeObj);
  JIM_GET_API(InvalidateStringRep);
  JIM_GET_API(InitStringRep);
  JIM_GET_API(DuplicateObj);
  JIM_GET_API(GetString);
  JIM_GET_API(Length);
  JIM_GET_API(InvalidateStringRep);
  JIM_GET_API(NewStringObj);
  JIM_GET_API(NewStringObjNoAlloc);
  JIM_GET_API(AppendString);
  JIM_GET_API(AppendString_sprintf);
  JIM_GET_API(AppendObj);
  JIM_GET_API(AppendStrings);
  JIM_GET_API(StringEqObj);
  JIM_GET_API(StringMatchObj);
  JIM_GET_API(StringRangeObj);
  JIM_GET_API(FormatString);
  JIM_GET_API(ScanString);
  JIM_GET_API(CompareStringImmediate);
  JIM_GET_API(NewReference);
  JIM_GET_API(GetReference);
  JIM_GET_API(SetFinalizer);
  JIM_GET_API(GetFinalizer);
  JIM_GET_API(CreateInterp);
  JIM_GET_API(FreeInterp);
  JIM_GET_API(GetExitCode);
  JIM_GET_API(SetStdin);
  JIM_GET_API(SetStdout);
  JIM_GET_API(SetStderr);
  JIM_GET_API(CreateCommand);
  JIM_GET_API(CreateProcedure);
  JIM_GET_API(DeleteCommand);
  JIM_GET_API(RenameCommand);
  JIM_GET_API(GetCommand);
  JIM_GET_API(SetVariable);
  JIM_GET_API(SetVariableStr);
  JIM_GET_API(SetGlobalVariableStr);
  JIM_GET_API(SetVariableStrWithStr);
  JIM_GET_API(SetVariableLink);
  JIM_GET_API(GetVariable);
  JIM_GET_API(GetCallFrameByLevel);
  JIM_GET_API(Collect);
  JIM_GET_API(CollectIfNeeded);
  JIM_GET_API(GetIndex);
  JIM_GET_API(NewListObj);
  JIM_GET_API(ListInsertElements);
  JIM_GET_API(ListAppendElement);
  JIM_GET_API(ListAppendList);
  JIM_GET_API(ListLength);
  JIM_GET_API(ListIndex);
  JIM_GET_API(SetListIndex);
  JIM_GET_API(ConcatObj);
  JIM_GET_API(NewDictObj);
  JIM_GET_API(DictKey);
  JIM_GET_API(DictKeysVector);
  JIM_GET_API(GetIndex);
  JIM_GET_API(GetReturnCode);
  JIM_GET_API(EvalExpression);
  JIM_GET_API(GetBoolFromExpr);
  JIM_GET_API(GetWide);
  JIM_GET_API(GetLong);
  JIM_GET_API(SetWide);
  JIM_GET_API(NewIntObj);
  JIM_GET_API(GetDouble);
  JIM_GET_API(SetDouble);
  JIM_GET_API(NewDoubleObj);
  JIM_GET_API(WrongNumArgs);
  JIM_GET_API(SetDictKeysVector);
  JIM_GET_API(SubstObj);
  JIM_GET_API(RegisterApi);
  JIM_GET_API(PrintErrorMessage);
  JIM_GET_API(InteractivePrompt);
  JIM_GET_API(RegisterCoreCommands);
  JIM_GET_API(GetSharedString);
  JIM_GET_API(ReleaseSharedString);
  JIM_GET_API(Panic);
  JIM_GET_API(StrDup);
  JIM_GET_API(UnsetVariable);
  JIM_GET_API(GetVariableStr);
  JIM_GET_API(GetGlobalVariable);
  JIM_GET_API(GetGlobalVariableStr);
  JIM_GET_API(GetAssocData);
  JIM_GET_API(SetAssocData);
  JIM_GET_API(DeleteAssocData);
  JIM_GET_API(GetEnum);
  JIM_GET_API(GetNvp);
  JIM_GET_API(ScriptIsComplete);
  JIM_GET_API(PackageProvide);
  JIM_GET_API(PackageRequire);
  JIM_GET_API(InitStack);
  JIM_GET_API(FreeStack);
  JIM_GET_API(StackLen);
  JIM_GET_API(StackPush);
  JIM_GET_API(StackPop);
  JIM_GET_API(StackPeek);
  JIM_GET_API(FreeStackElements);
  JIM_GET_API(fprintf  );
  JIM_GET_API(vfprintf );
  JIM_GET_API(fwrite   );
  JIM_GET_API(fread    );
  JIM_GET_API(fflush   );
  JIM_GET_API(fgets    );
  JIM_GET_API(Nvp_name2value);
  JIM_GET_API(Nvp_name2value_nocase);
  JIM_GET_API(Nvp_name2value_simple);
  
  JIM_GET_API(Nvp_value2name);
  JIM_GET_API(Nvp_value2name_simple);


  JIM_GET_API(Nvp_name2value_obj);
  JIM_GET_API(Nvp_value2name_obj);
  JIM_GET_API(Nvp_name2value_obj_nocase);

  JIM_GET_API(GetOpt_Setup);
  JIM_GET_API(GetOpt_Obj);
  JIM_GET_API(GetOpt_String);
  JIM_GET_API(GetOpt_Double);
  JIM_GET_API(GetOpt_Wide);
  JIM_GET_API(GetOpt_Nvp);
  JIM_GET_API(GetOpt_NvpUnknown);
  JIM_GET_API(GetOpt_Enum);
  JIM_GET_API(GetOpt_Debug);
  JIM_GET_API(SetResult_sprintf);
  JIM_GET_API(SetResult_NvpUnknown);
  JIM_GET_API(Debug_ArgvString);
}
#endif /* defined JIM_EXTENSION || defined JIM_EMBEDDED */

#undef JIM_GET_API

#ifdef JIM_EMBEDDED
Jim_Interp *ExportedJimCreateInterp(void);
static __inline__ void Jim_InitEmbedded(void) {
    Jim_Interp *i = ExportedJimCreateInterp();
    Jim_InitExtension(i);
    Jim_FreeInterp(i);
}
#endif /* JIM_EMBEDDED */
#endif /* __JIM_CORE__ */

#ifdef __cplusplus
}
#endif

#endif /* __JIM__H */

/*
 * Local Variables: ***
 * c-basic-offset: 4 ***
 * tab-width: 4 ***
 * End: ***
 */
