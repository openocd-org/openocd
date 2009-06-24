#ifndef HELPER_MEMBUF_H
#define HELPER_MEMBUF_H

/** @file */

/** @page MEMBUF - an auto-growing string buffer
 *
 * With OpenOCD often, one must write code that sends text to
 * different places.. the historical command_ctx, or JIM output,
 * and/or other places.
 *
 * This is a simple 'string buffer' that auto-grows.
 *
 * More correctly put, this is a "memory buffer"
 * it may contain binary data 
 * 
 * Note: Internally the buffer always has a 'null terminator'
 */

/* contents of this structure are 'opaque' */
struct membuf;
  

/** Create a new membuf
 * By default the memory buffer has "some non-zero-size"
 * (couple hundred bytes, exact amount is opaque)
 */
struct membuf *membuf_new(void);

/** delete (destroy) the mem buffer
 * @param pBuf - buffer to release
 */
void membuf_delete( struct membuf *pBuf );


/** grow/shrink a membuf by specified amount.
 * @param pBuf   - the buffer 
 * @param amount - the amount to grow or shrink by.
 *
 * Symantics of 'realloc()' return NULL on failure
 */
struct membuf *membuf_grow( struct membuf *pBuf, int amount );

/** how long is this buffer (memlen(), strlen())
 * @param pBuf - the buffer
 * 
 * @returns: length of current buffer.
 */
size_t membuf_len( struct membuf *pBuf );


/** reset an membuf to zero length.
 * @param pBuf - buffer to reset
 *
 * Note this does not 'release' the memory buffer
 */
void membuf_reset( struct membuf *pBuf );


/** sprintf() to the string buffer
 * @param pBuf - buffer to capture sprintf() data into
 * @param fmt  - printf format
 *
 * Returns 0 on success
 * Returns non-zero on failure
 */
int membuf_sprintf( struct membuf *pBuf , const char *fmt, ... );

/** vsprintf() to the string buffer
 * @param pBuf - buffer to capture sprintf() data into
 * @param fmt  - printf format
 * @param ap   - va_list for fmt
 *
 * Returns 0 on success
 * Returns non-zero on failure
 */
int membuf_vsprintf( struct membuf *pBuf , const char *fmt, va_list ap);

/** Tokenize lines using strtok() 
 * @param pBuf - buffer to tokenize
 * @param delim - delimiter parameter for strtok_r()
 * 
 * Identical to "strtok()" - pass "pBuff=NULL" on second call
 *
 * NOTE: This call is <b>destructive</b> to the buffer.
 */
const char *membuf_strtok( struct membuf *pBuf, const char *delim, void **pSave );

/** Return pointer to the memory in the buffer
 * @param pBuf - buffer
 *
 * NOTE: Thou shall not modify this pointer, it is <b>CONST</b>
 */
const void *membuf_datapointer( struct membuf *pBuf );


/** Append data to the buffer
 * @param pBuf  - buffer to append
 * @param pData - pointer to data to append
 * @param len   - length of data to append
 *
 * Modified symantics of "memcpy()".  On memory allocation failure
 * returns NULL.  On success, returns pointer to orginal membuf.
 */
struct membuf *membuf_append( struct membuf *pBuf, const void *pData, size_t len );


/** Append string to the buffer
 * @param pBuf  - buffer to append
 * @param str   - string to append
 *
 * Modified symantics of "strcat()".  On memory allocation failure
 * returns NULL.  On success, returns pointer to orginal membuf.
 */
struct membuf *membuf_strcat( struct membuf *pBuf, const char *s );


#endif
