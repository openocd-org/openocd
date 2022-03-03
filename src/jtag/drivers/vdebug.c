/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */
/*----------------------------------------------------------------------------
 * Copyright 2020-2021 Cadence Design Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *----------------------------------------------------------------------------
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *----------------------------------------------------------------------------
*/

/*!
 * @file
 *
 * @brief the virtual debug interface provides a connection between a sw debugger
 * and the simulated, emulated core over a soft connection, implemented by DPI
 * The vdebug debug driver currently supports JTAG transport
 * TODO: implement support and test big endian platforms
 *
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#ifdef HAVE_UNISTD_H
#include <unistd.h>          /* close */
#endif
#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif
#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif
#ifdef HAVE_NETDB_H
#include <netdb.h>
#endif
#endif
#include <stdio.h>
#ifdef HAVE_STDINT_H
#include <stdint.h>
#endif
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include "jtag/interface.h"
#include "jtag/commands.h"
#include "transport/transport.h"
#include "helper/time_support.h"
#include "helper/replacements.h"
#include "helper/log.h"

#define VD_VERSION 43
#define VD_BUFFER_LEN 4024
#define VD_CHEADER_LEN 24
#define VD_SHEADER_LEN 16

#define VD_MAX_MEMORIES 4
#define VD_POLL_INTERVAL 500
#define VD_SCALE_PSTOMS 1000000000

/**
 * @brief List of transactor types
 */
enum {
	VD_BFM_JTDP   = 0x0001,  /* transactor DAP JTAG DP */
	VD_BFM_SWDP   = 0x0002,  /* transactor DAP SWD DP */
	VD_BFM_AHB    = 0x0003,  /* transactor AMBA AHB */
	VD_BFM_APB    = 0x0004,  /* transactor AMBA APB */
	VD_BFM_AXI    = 0x0005,  /* transactor AMBA AXI */
	VD_BFM_JTAG   = 0x0006,  /* transactor serial JTAG */
	VD_BFM_SWD    = 0x0007,  /* transactor serial SWD */
};

/**
 * @brief List of signals that can be read or written by the debugger
 */
enum {
	VD_SIG_TCK    = 0x0001,  /* JTAG clock; tclk */
	VD_SIG_TDI    = 0x0002,  /* JTAG TDI;   tdi */
	VD_SIG_TMS    = 0x0004,  /* JTAG TMS;   tms */
	VD_SIG_RESET  = 0x0008,  /* DUT reset;  rst */
	VD_SIG_TRST   = 0x0010,  /* JTAG Reset; trstn */
	VD_SIG_TDO    = 0x0020,  /* JTAG TDO;   tdo */
	VD_SIG_POWER  = 0x0100,  /* BFM power;  bfm_up */
	VD_SIG_TCKDIV = 0x0200,  /* JTAG clock divider; tclkdiv */
	VD_SIG_BUF    = 0x1000,  /* memory buffer; mem */
};

/**
 * @brief List of errors
 */
enum {
	VD_ERR_NONE       = 0x0000,  /* no error */
	VD_ERR_NOT_IMPL   = 0x0100,  /* feature not implemented */
	VD_ERR_USAGE      = 0x0101,  /* incorrect usage */
	VD_ERR_PARAM      = 0x0102,  /* incorrect parameter */
	VD_ERR_CONFIG     = 0x0107,  /* incorrect configuration */
	VD_ERR_NO_MEMORY  = 0x0104,  /* out of memory */
	VD_ERR_SHM_OPEN   = 0x010a,  /* cannot open shared memory */
	VD_ERR_SHM_MAP    = 0x010b,  /* cannot map shared memory */
	VD_ERR_SOC_OPEN   = 0x011a,  /* cannot open socket */
	VD_ERR_SOC_OPT    = 0x011b,  /* cannot set socket option */
	VD_ERR_SOC_ADDR   = 0x011c,  /* cannot resolve host address */
	VD_ERR_SOC_CONN   = 0x011d,  /* cannot connect to host */
	VD_ERR_SOC_SEND   = 0x011e,  /* error sending data on socket */
	VD_ERR_SOC_RECV   = 0x011f,  /* error receiving data from socket */
	VD_ERR_LOCKED     = 0x0202,  /* device locked */
	VD_ERR_NOT_RUN    = 0x0204,  /* transactor not running */
	VD_ERR_NOT_OPEN   = 0x0205,  /* transactor not open/connected */
	VD_ERR_LICENSE    = 0x0206,  /* cannot check out the license */
	VD_ERR_VERSION    = 0x0207,  /* transactor version mismatch */
	VD_ERR_TIME_OUT   = 0x0301,  /* time out, waiting */
	VD_ERR_NO_POWER   = 0x0302,  /* power out error */
	VD_ERR_BUS_ERROR  = 0x0304,  /* bus protocol error, like pslverr */
	VD_ERR_NO_ACCESS  = 0x0306,  /* no access to an object */
	VD_ERR_INV_HANDLE = 0x0307,  /* invalid object handle */
	VD_ERR_INV_SCOPE  = 0x0308,  /* invalid scope */
};

enum {
	VD_CMD_OPEN       = 0x01,
	VD_CMD_CLOSE      = 0x02,
	VD_CMD_CONNECT    = 0x04,
	VD_CMD_DISCONNECT = 0x05,
	VD_CMD_WAIT       = 0x09,
	VD_CMD_SIGSET     = 0x0a,
	VD_CMD_SIGGET     = 0x0b,
	VD_CMD_JTAGCLOCK  = 0x0f,
	VD_CMD_JTAGSHTAP  = 0x1a,
	VD_CMD_MEMOPEN    = 0x21,
	VD_CMD_MEMCLOSE   = 0x22,
	VD_CMD_MEMWRITE   = 0x23,
};

enum {
	VD_BATCH_NO       = 0,
	VD_BATCH_WO       = 1,
	VD_BATCH_WR       = 2,
};

struct vd_shm {
	struct {                     /* VD_CHEADER_LEN written by client */
		uint8_t cmd;             /* 000; command */
		uint8_t type;            /* 001; interface type */
		uint16_t waddr;          /* 002; write pointer */
		uint16_t wbytes;         /* 004; data bytes */
		uint16_t rbytes;         /* 006; data bytes to read */
		uint16_t wwords;         /* 008; data words */
		uint16_t rwords;         /* 00a; data words to read */
		uint32_t rwdata;         /* 00c; read/write data */
		uint32_t offset;         /* 010; address offset */
		uint16_t offseth;        /* 014; address offset 47:32 */
		uint16_t wid;            /* 016; request id*/
	};
	union {                      /* 018; */
		uint8_t wd8[VD_BUFFER_LEN];
		uint16_t wd16[VD_BUFFER_LEN / 2];
		uint32_t wd32[VD_BUFFER_LEN / 4];
		uint64_t wd64[VD_BUFFER_LEN / 8];
	};
	struct {                     /* VD_SHEADER_LEN written by server */
		uint16_t rid;            /* fd0: request id read */
		uint16_t awords;         /* fd2: actual data words read back */
		int32_t  status;         /* fd4; */
		uint64_t duttime;        /* fd8; */
	};
	union {                      /* fe0: */
		uint8_t rd8[VD_BUFFER_LEN];
		uint16_t rd16[VD_BUFFER_LEN / 2];
		uint32_t rd32[VD_BUFFER_LEN / 4];
		uint64_t rd64[VD_BUFFER_LEN / 8];
	};
	uint32_t state;              /* 1f98; connection state */
	uint32_t count;              /* 1f9c; */
	uint8_t dummy[96];           /* 1fa0; 48+40B+8B; */
};

struct vd_client {
	uint8_t trans_batch;
	bool trans_first;
	bool trans_last;
	uint8_t mem_ndx;
	uint8_t buf_width;
	uint8_t addr_bits;
	uint8_t bfm_type;
	uint16_t sig_read;
	uint16_t sig_write;
	uint32_t bfm_period;
	uint32_t mem_base[VD_MAX_MEMORIES];
	uint32_t mem_size[VD_MAX_MEMORIES];
	uint32_t mem_width[VD_MAX_MEMORIES];
	uint32_t mem_depth[VD_MAX_MEMORIES];
	uint16_t server_port;
	uint32_t poll_cycles;
	uint32_t poll_min;
	uint32_t poll_max;
	uint32_t targ_time;
	int hsocket;
	char server_name[32];
	char bfm_path[128];
	char mem_path[VD_MAX_MEMORIES][128];
	uint8_t *tdo;
};

struct vd_jtag_hdr {
	uint64_t tlen:24;
	uint64_t post:3;
	uint64_t pre:3;
	uint64_t cmd:2;
	uint64_t wlen:16;
	uint64_t rlen:16;
};

static struct vd_shm *pbuf;
static struct vd_client vdc;

static int vdebug_socket_error(void)
{
#ifdef _WIN32
	return WSAGetLastError();
#else
	return errno;
#endif
}

static int vdebug_socket_open(char *server_addr, uint32_t port)
{
	int hsock;
	int rc = 0;
	uint32_t buflen = sizeof(struct vd_shm); /* size of the send and rcv buffer */
	struct addrinfo *ainfo = NULL;
	struct addrinfo ahint = { 0, AF_INET, SOCK_STREAM, 0, 0, NULL, NULL, NULL };

#ifdef _WIN32
	hsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (hsock == INVALID_SOCKET)
		rc = vdebug_socket_error();
#else
	uint32_t rcvwat = VD_SHEADER_LEN;    /* size of the rcv header, as rcv min watermark */
	hsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (hsock < 0)
		rc = errno;
	else if (setsockopt(hsock, SOL_SOCKET, SO_RCVLOWAT, &rcvwat, sizeof(rcvwat)) < 0)
		rc = errno;
#endif
	else if (setsockopt(hsock, SOL_SOCKET, SO_SNDBUF, (const char *)&buflen, sizeof(buflen)) < 0)
		rc = vdebug_socket_error();
	else if (setsockopt(hsock, SOL_SOCKET, SO_RCVBUF, (const char *)&buflen, sizeof(buflen)) < 0)
		rc = vdebug_socket_error();

	if (rc) {
		LOG_ERROR("socket_open: cannot set socket option, error %d", rc);
	} else if (getaddrinfo(server_addr, NULL, &ahint, &ainfo) != 0) {
		LOG_ERROR("socket_open: cannot resolve address %s, error %d", server_addr, vdebug_socket_error());
		rc = VD_ERR_SOC_ADDR;
	} else {
		((struct sockaddr_in *)(ainfo->ai_addr))->sin_port = htons(port);
		if (connect(hsock, ainfo->ai_addr, sizeof(struct sockaddr)) < 0) {
			LOG_ERROR("socket_open: cannot connect to %s:%d, error %d", server_addr, port, vdebug_socket_error());
			rc = VD_ERR_SOC_CONN;
		}
	}

	if (rc) {
		close_socket(hsock);
		hsock = 0;
	}

	if (ainfo)
		freeaddrinfo(ainfo);

	return hsock;
}

static int vdebug_socket_receive(int hsock, struct vd_shm *pmem)
{
	int rc;
	int dreceived = 0;
	int offset = (uint8_t *)&pmem->rid - &pmem->cmd;
	int to_receive = VD_SHEADER_LEN + pmem->rbytes;
	char *pb = (char *)pmem;

	do {
		rc = recv(hsock, pb + offset, to_receive, 0);
		if (rc <= 0) {
			LOG_WARNING("socket_receive: recv failed, error %d", rc < 0 ? vdebug_socket_error() : 0);
			return rc;
		}
		to_receive -= rc;
		offset += rc;
		LOG_DEBUG_IO("socket_receive: received %d, to receive %d", rc, to_receive);
		dreceived += rc;
	} while (to_receive);

	return dreceived;
}

static int vdebug_socket_send(int hsock, struct vd_shm *pmem)
{
	int rc = send(hsock, (const char *)&pmem->cmd, VD_CHEADER_LEN + pmem->wbytes, 0);
	if (rc <= 0)
		LOG_WARNING("socket_send: send failed, error %d", vdebug_socket_error());
	else
		LOG_DEBUG_IO("socket_send: sent %d, to send 0", rc);

	return rc;
}

static uint32_t vdebug_wait_server(int hsock, struct vd_shm *pmem)
{
	if (!hsock)
		return VD_ERR_SOC_OPEN;
	int st = vdebug_socket_send(hsock, pmem);
	if (st <= 0)
		return VD_ERR_SOC_SEND;

	int rd = vdebug_socket_receive(hsock, pmem);
	if (rd  <= 0)
		return VD_ERR_SOC_RECV;

	int rc = pmem->status;
	LOG_DEBUG_IO("wait_server: cmd %02" PRIx8 " done, sent %d, rcvd %d, status %d",
					pmem->cmd, st, rd, rc);

	return rc;
}

int vdebug_run_jtag_queue(int hsock, struct vd_shm *pm, unsigned int count)
{
	uint8_t  num_pre, num_post, tdi, tms;
	unsigned int num, anum, bytes, hwords, words;
	unsigned int req, waddr, rwords;
	int64_t ts, te;
	uint8_t *tdo;
	int rc;
	struct vd_jtag_hdr *hdr;

	req = 0;                            /* beginning of request */
	waddr = 0;
	rwords = 0;
	pm->wbytes = pm->wwords * vdc.buf_width;
	pm->rbytes = pm->rwords * vdc.buf_width;
	ts = timeval_ms();
	rc = vdebug_wait_server(hsock, pm);
	while (!rc && (req < count)) {      /* loop over requests to read data and print out */
		hdr = (struct vd_jtag_hdr *)&pm->wd8[waddr * 4];
		hwords = hdr->wlen;
		words = hdr->rlen;
		anum = hdr->tlen;
		num_pre = hdr->pre;
		num_post = hdr->post;
		if (num_post)
			num = anum - num_pre - num_post + 1;
		else
			num = anum - num_pre;
		bytes = (num + 7) / 8;
		vdc.trans_last = (req + 1) < count ? 0 : 1;
		vdc.trans_first = waddr ? 0 : 1;
		if (hdr->cmd == 3) { /* read */
			tdo = vdc.tdo;
			for (unsigned int j = 0; j < bytes; j++) {
				tdo[j] = (pm->rd8[rwords * 8 + j] >> num_pre) | (pm->rd8[rwords * 8 + j + 1] << (8 - num_pre));
				LOG_DEBUG_IO("%04x D0[%02x]:%02x", pm->wid - count + req, j, tdo[j]);
			}
			rwords += words;           /* read data offset */
		} else {
			tdo = NULL;
		}
		waddr += sizeof(struct vd_jtag_hdr) / 4; /* waddr past header */
		tdi = (pm->wd8[waddr * 4] >> num_pre) | (pm->wd8[waddr * 4 + 1] << (8 - num_pre));
		tms = (pm->wd8[waddr * 4 + 4] >> num_pre) | (pm->wd8[waddr * 4 + 4 + 1] << (8 - num_pre));
		LOG_DEBUG("%04x L:%02d O:%05x @%03x DI:%02x MS:%02x DO:%02x",
			pm->wid - count + req, num, (vdc.trans_first << 14) | (vdc.trans_last << 15),
			waddr - 2, tdi, tms, (tdo ? tdo[0] : 0xdd));
		waddr += hwords * 2;           /* start of next request */
		req += 1;
	}

	if (rc) {
		LOG_ERROR("0x%x executing transaction", rc);
		rc = ERROR_FAIL;
	}

	te = timeval_ms();
	vdc.targ_time += (uint32_t)(te - ts);
	pm->offseth = 0;     /* reset buffer write address */
	pm->offset = 0;
	pm->rwords = 0;
	pm->waddr = 0;

	return rc;
}

static int vdebug_open(int hsock, struct vd_shm *pm, const char *path,
						uint8_t type, uint32_t period_ps, uint32_t sig_mask)
{
	int rc = VD_ERR_NOT_OPEN;

	pm->cmd = VD_CMD_OPEN;
	pm->wid = VD_VERSION;              /* client version */
	pm->wbytes = 0;
	pm->rbytes = 0;
	pm->wwords = 0;
	pm->rwords = 0;
	rc = vdebug_wait_server(hsock, pm);
	if (rc != 0) {                     /* communication problem */
		LOG_ERROR("0x%x connecting to server", rc);
	} else if (pm->rid < pm->wid) {
		LOG_ERROR("server version %d too old for the client %d", pm->rid, pm->wid);
		pm->cmd = VD_CMD_CLOSE;        /* let server close the connection */
		vdebug_wait_server(hsock, pm);
		rc = VD_ERR_VERSION;
	} else {
		pm->cmd = VD_CMD_CONNECT;
		pm->type = type;               /* BFM type to connect to, here JTAG */
		pm->rwdata = sig_mask | VD_SIG_BUF | (VD_SIG_BUF << 16);
		pm->wbytes = strlen(path) + 1;
		pm->rbytes = 12;
		pm->wid = 0;              /* reset wid for transaction ID */
		pm->wwords = 0;
		pm->rwords = 0;
		memcpy(pm->wd8, path, pm->wbytes + 1);
		rc = vdebug_wait_server(hsock, pm);
		vdc.sig_read = pm->rwdata >> 16;  /* signal read mask */
		vdc.sig_write = pm->rwdata;     /* signal write mask */
		vdc.bfm_period = period_ps;
		vdc.buf_width = pm->rd32[0] / 8;/* access width in bytes */
		vdc.addr_bits = pm->rd32[2];    /* supported address bits */
	}

	if (rc) {
		LOG_ERROR("0x%x connecting to BFM %s", rc, path);
		return ERROR_FAIL;
	}

	LOG_DEBUG("%s type %0x, period %dps, buffer %dx%dB signals r%04xw%04x",
		path, type, vdc.bfm_period, VD_BUFFER_LEN / vdc.buf_width,
		vdc.buf_width, vdc.sig_read, vdc.sig_write);

	return ERROR_OK;
}

static int vdebug_close(int hsock, struct vd_shm *pm, uint8_t type)
{
	pm->cmd = VD_CMD_DISCONNECT;
	pm->type = type;              /* BFM type, here JTAG */
	pm->wbytes = 0;
	pm->rbytes = 0;
	pm->wwords = 0;
	pm->rwords = 0;
	vdebug_wait_server(hsock, pm);
	pm->cmd = VD_CMD_CLOSE;
	pm->wid = VD_VERSION;    /* client version */
	pm->wbytes = 0;
	pm->rbytes = 0;
	pm->wwords = 0;
	pm->rwords = 0;
	vdebug_wait_server(hsock, pm);
	LOG_DEBUG("type %0x", type);

	return ERROR_OK;
}

static int vdebug_wait(int hsock, struct vd_shm *pm, uint32_t cycles)
{
	if (cycles) {
		pm->cmd = VD_CMD_WAIT;
		pm->wbytes = 0;
		pm->rbytes = 0;
		pm->rwdata = cycles;  /* clock sycles to wait */
		int rc = vdebug_wait_server(hsock, pm);
		if (rc) {
			LOG_ERROR("0x%x waiting %" PRIx32 " cycles", rc, cycles);
			return ERROR_FAIL;
		}
		LOG_DEBUG("%d cycles", cycles);
	}

	return ERROR_OK;
}

static int vdebug_sig_set(int hsock, struct vd_shm *pm, uint32_t write_mask, uint32_t value)
{
	pm->cmd = VD_CMD_SIGSET;
	pm->wbytes = 0;
	pm->rbytes = 0;
	pm->rwdata = (write_mask << 16) | (value & 0xffff); /* mask and value of signals to set */
	int rc = vdebug_wait_server(hsock, pm);
	if (rc) {
		LOG_ERROR("0x%x setting signals %04" PRIx32, rc, write_mask);
		return ERROR_FAIL;
	}

	LOG_DEBUG("setting signals %04" PRIx32 " to %04" PRIx32, write_mask, value);

	return ERROR_OK;
}

static int vdebug_jtag_clock(int hsock, struct vd_shm *pm, uint32_t value)
{
	pm->cmd = VD_CMD_JTAGCLOCK;
	pm->wbytes = 0;
	pm->rbytes = 0;
	pm->rwdata = value;  /* divider value */
	int rc = vdebug_wait_server(hsock, pm);
	if (rc) {
		LOG_ERROR("0x%x setting jtag_clock", rc);
		return ERROR_FAIL;
	}

	LOG_DEBUG("setting jtag clock divider to %" PRIx32, value);

	return ERROR_OK;
}

static int vdebug_jtag_shift_tap(int hsock, struct vd_shm *pm, uint8_t num_pre,
								 const uint8_t tms_pre, uint32_t num, const uint8_t *tdi,
								 uint8_t num_post, const uint8_t tms_post, uint8_t *tdo,
								 uint8_t f_last)
{
	const uint32_t tobits = 8;
	uint16_t bytes, hwords, anum, words, waddr;
	int rc = 0;

	pm->cmd = VD_CMD_JTAGSHTAP;
	vdc.trans_last = f_last || (vdc.trans_batch == VD_BATCH_NO) || tdo;
	if (vdc.trans_first)
		waddr = 0;             /* reset buffer offset */
	else
		waddr = pm->offseth;   /* continue from the previous transaction */
	if (num_post)          /* actual number of bits to shift */
		anum = num + num_pre + num_post - 1;
	else
		anum = num + num_pre;
	hwords = (anum + 4 * vdc.buf_width - 1) / (4 * vdc.buf_width); /* in 4B TDI/TMS words */
	words = (hwords + 1) / 2;    /* in 8B TDO words to read */
	bytes = (num + 7) / 8;       /* data only portion in bytes */
	/* buffer overflow check and flush */
	if (4 * waddr + sizeof(struct vd_jtag_hdr) + 8 * hwords + 64 > VD_BUFFER_LEN) {
		vdc.trans_last = 1;        /* force flush within 64B of buffer end */
	} else if (4 * waddr + sizeof(struct vd_jtag_hdr) + 8 * hwords > VD_BUFFER_LEN) {
		/* this req does not fit, discard it */
		LOG_ERROR("%04x L:%02d O:%05x @%04x too many bits to shift",
			pm->wid, anum, (vdc.trans_first << 14) | (vdc.trans_last << 15), waddr);
		rc = ERROR_FAIL;
	}

	if (!rc && anum) {
		uint16_t i, j;
		struct vd_jtag_hdr *hdr = (struct vd_jtag_hdr *)&pm->wd8[4 * waddr]; /* 8 bytes header */
		hdr->cmd = (tdo ? 3 : 1); /* R and W bits */
		hdr->pre = num_pre;
		hdr->post = num_post;
		hdr->tlen = anum;
		hdr->wlen = hwords;
		hdr->rlen = words;
		pm->wid++;               /* transaction ID */
		waddr += 2;              /* waddr past header */
		/* TDI/TMS data follows as 32 bit word pairs {TMS,TDI} */
		pm->wd8[4 * waddr] = (tdi ? (tdi[0] << num_pre) : 0);
		pm->wd8[4 * waddr + 4] = tms_pre;    /* init with tms_pre */
		if (num + num_pre <= 8)            /* and tms_post for num <=4 */
			pm->wd8[4 * waddr + 4] |= (tms_post << (num + num_pre - 1));
		for (i = 1, j = 4 * waddr; i < bytes; i++) {
			if (i == bytes - 1 && num + num_pre <= bytes * tobits)
				pm->wd8[j + i + 4] = tms_post << ((num + num_pre - 1) % 8);
			else
				pm->wd8[j + i + 4] = 0x0;/* placing 4 bytes of TMS bits into high word */
			if (!tdi)             /* placing 4 bytes of TDI bits into low word */
				pm->wd8[j + i] = 0x0;
			else
				pm->wd8[j + i] = (tdi[i] << num_pre) | (tdi[i - 1] >> (8 - num_pre));
			if (i % 4 == 3)
				j += 4;
		}

		if (tdi)
			if (num + num_pre > bytes * tobits) /* in case 1 additional byte needed for TDI */
				pm->wd8[j + i] = (tdi[i - 1] >> (8 - num_pre)); /* put last TDI bits there */

		if (num + num_pre <= bytes * tobits) { /* in case no or 1 additional byte needed */
			pm->wd8[j + i + 4] = tms_post >> (8 - (num + num_pre - 1) % 8); /* may need to add higher part */
		/* in case exactly 1 additional byte needed */
		} else if (num + num_pre > bytes * tobits && anum <= (bytes + 1) * tobits) {
			pm->wd8[j + i + 4] = tms_post << ((num + num_pre - 1) % 8); /* add whole tms_post */
		} else {                           /* in case 2 additional bytes, tms_post split */
			pm->wd8[j + i + 4] = tms_post << ((num + num_pre - 1) % 8);/* add lower part of tms_post */
			if (i % 4 == 3)              /* next byte is in the next 32b word */
				pm->wd8[j + i + 4 + 5] = tms_post >> (8 - (num + num_pre - 1) % 8); /* and higher part */
			else                         /* next byte is in the same 32b word */
				pm->wd8[j + i + 4 + 1] = tms_post >> (8 - (num + num_pre - 1) % 8); /* and higher part */
		}

		if (tdo) {
			pm->rwords += words;       /* keep track of the words to read */
			vdc.tdo = tdo;
		}
		pm->wwords = waddr / 2 + hwords;   /* payload size *2 to include both TDI and TMS data */
		pm->waddr++;
	}

	if (!waddr)                        /* flush issued, but buffer empty */
		;
	else if (!vdc.trans_last)          /* buffered request */
		pm->offseth = waddr + hwords * 2;  /* offset for next transaction, must be even */
	else                               /* execute batch of requests */
		rc = vdebug_run_jtag_queue(hsock, pm, pm->waddr);
	vdc.trans_first = vdc.trans_last; /* flush forces trans_first flag */

	return rc;
}

static int vdebug_mem_open(int hsock, struct vd_shm *pm, const char *path, uint8_t ndx)
{
	int rc;

	if (!path)
		return ERROR_OK;

	pm->cmd = VD_CMD_MEMOPEN;
	pm->wbytes = strlen(path) + 1;   /* includes terminating 0 */
	pm->rbytes = 8;
	pm->wwords = 0;
	pm->rwords = 0;
	memcpy(pm->wd8, path, pm->wbytes);
	rc = vdebug_wait_server(hsock, pm);
	if (rc) {
		LOG_ERROR("0x%x opening memory %s", rc, path);
	} else if (ndx != pm->rd16[1]) {
		LOG_WARNING("Invalid memory index %" PRIu16 " returned. Direct memory access disabled", pm->rd16[1]);
	} else {
		vdc.mem_width[ndx] = pm->rd16[0] / 8;   /* memory width in bytes */
		vdc.mem_depth[ndx] = pm->rd32[1];       /* memory depth in words */
		LOG_DEBUG("%" PRIx8 ": %s memory %" PRIu32 "x%" PRIu32 "B, buffer %" PRIu32 "x%" PRIu32 "B", ndx, path,
			vdc.mem_depth[ndx], vdc.mem_width[ndx], VD_BUFFER_LEN / vdc.mem_width[ndx], vdc.mem_width[ndx]);
	}

	return ERROR_OK;
}

static void vdebug_mem_close(int hsock, struct vd_shm *pm, uint8_t ndx)
{
	pm->cmd = VD_CMD_MEMCLOSE;
	pm->rwdata = ndx;        /* which memory */
	pm->wbytes = 0;
	pm->rbytes = 0;
	pm->wwords = 0;
	pm->rwords = 0;
	vdebug_wait_server(hsock, pm);
	LOG_DEBUG("%" PRIx8 ": %s", ndx, vdc.mem_path[ndx]);
}

static int vdebug_init(void)
{
	vdc.hsocket = vdebug_socket_open(vdc.server_name, vdc.server_port);
	pbuf = calloc(1, sizeof(struct vd_shm));
	if (!pbuf) {
		close_socket(vdc.hsocket);
		vdc.hsocket = 0;
		LOG_ERROR("cannot allocate %lu bytes", sizeof(struct vd_shm));
		return ERROR_FAIL;
	}
	if (vdc.hsocket <= 0) {
		free(pbuf);
		pbuf = NULL;
		LOG_ERROR("cannot connect to vdebug server %s:%" PRIu16,
			vdc.server_name, vdc.server_port);
		return ERROR_FAIL;
	}
	vdc.trans_first = 1;
	vdc.poll_cycles = vdc.poll_max;
	uint32_t sig_mask = VD_SIG_RESET | VD_SIG_TRST | VD_SIG_TCKDIV;
	int rc = vdebug_open(vdc.hsocket, pbuf, vdc.bfm_path, vdc.bfm_type, vdc.bfm_period, sig_mask);
	if (rc != 0) {
		LOG_ERROR("cannot connect to %s, rc 0x%x", vdc.bfm_path, rc);
		close_socket(vdc.hsocket);
		vdc.hsocket = 0;
		free(pbuf);
		pbuf = NULL;
	} else {
		for (uint8_t i = 0; i < vdc.mem_ndx; i++) {
			rc = vdebug_mem_open(vdc.hsocket, pbuf, vdc.mem_path[i], i);
			if (rc != 0)
				LOG_ERROR("cannot connect to %s, rc 0x%x", vdc.mem_path[i], rc);
		}

		LOG_INFO("vdebug %d connected to %s through %s:%" PRIu16,
				 VD_VERSION, vdc.bfm_path, vdc.server_name, vdc.server_port);
	}

	return rc;
}

static int vdebug_quit(void)
{
	for (uint8_t i = 0; i < vdc.mem_ndx; i++)
		if (vdc.mem_width[i])
			vdebug_mem_close(vdc.hsocket, pbuf, i);
	int rc = vdebug_close(vdc.hsocket, pbuf, vdc.bfm_type);
	LOG_INFO("vdebug %d disconnected from %s through %s:%" PRIu16 " rc:%d", VD_VERSION,
		vdc.bfm_path, vdc.server_name, vdc.server_port, rc);
	if (vdc.hsocket)
		close_socket(vdc.hsocket);
	free(pbuf);
	pbuf = NULL;

	return ERROR_OK;
}

static int vdebug_reset(int trst, int srst)
{
	uint16_t sig_val = 0xffff;
	uint16_t sig_mask = 0;

	sig_mask |= VD_SIG_RESET;
	if (srst)
		sig_val &= ~VD_SIG_RESET;/* active low */
	if (transport_is_jtag()) {
		sig_mask |= VD_SIG_TRST;
		if (trst)
			sig_val &= ~VD_SIG_TRST; /* active low */
	}

	LOG_INFO("rst trst:%d srst:%d mask:%" PRIx16 " val:%" PRIx16, trst, srst, sig_mask, sig_val);
	int rc = vdebug_sig_set(vdc.hsocket, pbuf, sig_mask, sig_val);
	if (rc == 0)
		rc = vdebug_wait(vdc.hsocket, pbuf, 20); /* 20 clock cycles pulse */

	return rc;
}

static int vdebug_jtag_tms_seq(const uint8_t *tms, int num, uint8_t f_flush)
{
	LOG_INFO("tms  len:%d tms:%x", num, *(const uint32_t *)tms);

	return vdebug_jtag_shift_tap(vdc.hsocket, pbuf, num, *tms, 0, NULL, 0, 0, NULL, f_flush);
}

static int vdebug_jtag_path_move(struct pathmove_command *cmd, uint8_t f_flush)
{
	uint8_t tms[DIV_ROUND_UP(cmd->num_states, 8)];
	LOG_INFO("path num states %d", cmd->num_states);

	memset(tms, 0, DIV_ROUND_UP(cmd->num_states, 8));

	for (uint8_t i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			buf_set_u32(tms, i, 1, 1);
		tap_set_state(cmd->path[i]);
	}

	return vdebug_jtag_tms_seq(tms, cmd->num_states, f_flush);
}

static int vdebug_jtag_tlr(tap_state_t state, uint8_t f_flush)
{
	int rc = ERROR_OK;

	uint8_t cur = tap_get_state();
	uint8_t tms_pre = tap_get_tms_path(cur, state);
	uint8_t num_pre = tap_get_tms_path_len(cur, state);
	LOG_INFO("tlr  from %" PRIx8 " to %" PRIx8, cur, state);
	if (cur != state) {
		rc = vdebug_jtag_shift_tap(vdc.hsocket, pbuf, num_pre, tms_pre, 0, NULL, 0, 0, NULL, f_flush);
		tap_set_state(state);
	}

	return rc;
}

static int vdebug_jtag_scan(struct scan_command *cmd, uint8_t f_flush)
{
	int rc = ERROR_OK;

	uint8_t cur = tap_get_state();
	uint8_t state = cmd->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT;
	uint8_t tms_pre = tap_get_tms_path(cur, state);
	uint8_t num_pre = tap_get_tms_path_len(cur, state);
	uint8_t tms_post = tap_get_tms_path(state, cmd->end_state);
	uint8_t num_post = tap_get_tms_path_len(state, cmd->end_state);
	int num_bits = jtag_scan_size(cmd);
	LOG_DEBUG("scan len:%d fields:%d ir/!dr:%d state cur:%x end:%x",
			  num_bits, cmd->num_fields, cmd->ir_scan, cur, cmd->end_state);
	for (int i = 0; i < cmd->num_fields; i++) {
		uint8_t cur_num_pre = i == 0 ? num_pre : 0;
		uint8_t cur_tms_pre = i == 0 ? tms_pre : 0;
		uint8_t cur_num_post = i == cmd->num_fields - 1 ? num_post : 0;
		uint8_t cur_tms_post = i == cmd->num_fields - 1 ? tms_post : 0;
		uint8_t cur_flush = i == cmd->num_fields - 1 ? f_flush : 0;
		rc = vdebug_jtag_shift_tap(vdc.hsocket, pbuf, cur_num_pre, cur_tms_pre,
			cmd->fields[i].num_bits, cmd->fields[i].out_value, cur_num_post, cur_tms_post,
			cmd->fields[i].in_value, cur_flush);
		if (rc)
			break;
	}

	if (cur != cmd->end_state)
		tap_set_state(cmd->end_state);

	return rc;
}

static int vdebug_jtag_runtest(int cycles, tap_state_t state, uint8_t f_flush)
{
	uint8_t cur = tap_get_state();
	uint8_t tms_pre = tap_get_tms_path(cur, state);
	uint8_t num_pre = tap_get_tms_path_len(cur, state);
	LOG_DEBUG("idle len:%d state cur:%x end:%x", cycles, cur, state);
	int rc = vdebug_jtag_shift_tap(vdc.hsocket, pbuf, num_pre, tms_pre, cycles, NULL, 0, 0, NULL, f_flush);
	if (cur != state)
		tap_set_state(state);

	return rc;
}

static int vdebug_jtag_stableclocks(int num, uint8_t f_flush)
{
	LOG_INFO("stab len:%d state cur:%x", num, tap_get_state());

	return vdebug_jtag_shift_tap(vdc.hsocket, pbuf, 0, 0, num, NULL, 0, 0, NULL, f_flush);
}

static int vdebug_sleep(int us)
{
	LOG_INFO("sleep %d us", us);

	return vdebug_wait(vdc.hsocket, pbuf, us / 1000);
}

static int vdebug_jtag_speed(int speed)
{
	unsigned int clkmax = VD_SCALE_PSTOMS / (vdc.bfm_period * 2); /* kHz */
	unsigned int divval = clkmax / speed;
	LOG_INFO("jclk speed:%d kHz set, BFM divider %u", speed, divval);

	return vdebug_jtag_clock(vdc.hsocket, pbuf, divval);
}

static int vdebug_jtag_khz(int khz, int *jtag_speed)
{
	unsigned int clkmax = VD_SCALE_PSTOMS / (vdc.bfm_period * 2); /* kHz */
	unsigned int divval = khz ? clkmax / khz : 1;
	*jtag_speed = clkmax / divval;
	LOG_DEBUG("khz  speed:%d from khz:%d", *jtag_speed, khz);

	return ERROR_OK;
}

static int vdebug_jtag_div(int speed, int *khz)
{
	*khz = speed;
	LOG_DEBUG("div  khz:%d from speed:%d", *khz, speed);

	return ERROR_OK;
}

static int vdebug_jtag_execute_queue(void)
{
	int rc = ERROR_OK;

	for (struct jtag_command *cmd = jtag_command_queue; rc == ERROR_OK && cmd; cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RUNTEST:
			rc = vdebug_jtag_runtest(cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state, !cmd->next);
			break;
		case JTAG_STABLECLOCKS:
			rc = vdebug_jtag_stableclocks(cmd->cmd.stableclocks->num_cycles, !cmd->next);
			break;
		case JTAG_TLR_RESET:
			rc = vdebug_jtag_tlr(cmd->cmd.statemove->end_state, !cmd->next);
			break;
		case JTAG_PATHMOVE:
			rc = vdebug_jtag_path_move(cmd->cmd.pathmove, !cmd->next);
			break;
		case JTAG_TMS:
			rc = vdebug_jtag_tms_seq(cmd->cmd.tms->bits, cmd->cmd.tms->num_bits, !cmd->next);
			break;
		case JTAG_SLEEP:
			rc = vdebug_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			rc = vdebug_jtag_scan(cmd->cmd.scan, !cmd->next);
			break;
		default:
			LOG_ERROR("Unknown JTAG command type 0x%x encountered", cmd->type);
			rc = ERROR_FAIL;
		}
	}

	return rc;
}

COMMAND_HANDLER(vdebug_set_server)
{
	if ((CMD_ARGC != 1) || !strchr(CMD_ARGV[0], ':'))
		return ERROR_COMMAND_SYNTAX_ERROR;

	char *pchar = strchr(CMD_ARGV[0], ':');
	*pchar = '\0';
	strncpy(vdc.server_name, CMD_ARGV[0], sizeof(vdc.server_name) - 1);
	int port = atoi(++pchar);
	if (port < 0 || port > UINT16_MAX) {
		LOG_ERROR("invalid port number %d specified", port);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	vdc.server_port = port;
	LOG_DEBUG("server: %s port %u", vdc.server_name, vdc.server_port);

	return ERROR_OK;
}

COMMAND_HANDLER(vdebug_set_bfm)
{
	char prefix;

	if ((CMD_ARGC != 2) || (sscanf(CMD_ARGV[1], "%u%c", &vdc.bfm_period, &prefix) != 2))
		return ERROR_COMMAND_SYNTAX_ERROR;

	strncpy(vdc.bfm_path, CMD_ARGV[0], sizeof(vdc.bfm_path) - 1);
	switch (prefix) {
	case 'u':
		vdc.bfm_period *= 1000000;
		break;
	case 'n':
		vdc.bfm_period *= 1000;
		break;
	case 'p':
	default:
		break;
	}
	vdc.bfm_type = VD_BFM_JTAG;
	LOG_DEBUG("bfm_path: %s clk_period %ups", vdc.bfm_path, vdc.bfm_period);

	return ERROR_OK;
}

COMMAND_HANDLER(vdebug_set_mem)
{
	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (vdc.mem_ndx >= VD_MAX_MEMORIES) {
		LOG_ERROR("mem_path declared more than %d allowed times", VD_MAX_MEMORIES);
		return ERROR_FAIL;
	}

	strncpy(vdc.mem_path[vdc.mem_ndx], CMD_ARGV[0], sizeof(vdc.mem_path[vdc.mem_ndx]) - 1);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], vdc.mem_base[vdc.mem_ndx]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], vdc.mem_size[vdc.mem_ndx]);
	LOG_DEBUG("mem_path: set %s @ 0x%08x+0x%08x", vdc.mem_path[vdc.mem_ndx],
		vdc.mem_base[vdc.mem_ndx], vdc.mem_size[vdc.mem_ndx]);
	vdc.mem_ndx++;

	return ERROR_OK;
}

COMMAND_HANDLER(vdebug_set_batching)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (isdigit((unsigned char)CMD_ARGV[0][0]))
		vdc.trans_batch = (CMD_ARGV[0][0] == '0' ? 0 : (CMD_ARGV[0][0] == '1' ? 1 : 2));
	else if (CMD_ARGV[0][0] == 'r')
		vdc.trans_batch = VD_BATCH_WR;
	else if (CMD_ARGV[0][0] == 'w')
		vdc.trans_batch = VD_BATCH_WO;
	else
		vdc.trans_batch = VD_BATCH_NO;
	LOG_DEBUG("batching: set to %u", vdc.trans_batch);

	return ERROR_OK;
}

COMMAND_HANDLER(vdebug_set_polling)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	vdc.poll_min = atoi(CMD_ARGV[0]);
	vdc.poll_max = atoi(CMD_ARGV[1]);
	LOG_DEBUG("polling: set min %u max %u", vdc.poll_min, vdc.poll_max);

	return ERROR_OK;
}

static const struct command_registration vdebug_command_handlers[] = {
	{
		.name = "server",
		.handler = &vdebug_set_server,
		.mode = COMMAND_CONFIG,
		.help = "set the vdebug server name or address",
		.usage = "<host:port>",
	},
	{
		.name = "bfm_path",
		.handler = &vdebug_set_bfm,
		.mode = COMMAND_CONFIG,
		.help = "set the vdebug BFM hierarchical path",
		.usage = "<path> <clk_period[p|n|u]s>",
	},
	{
		.name = "mem_path",
		.handler = &vdebug_set_mem,
		.mode = COMMAND_ANY,
		.help = "set the design memory for the code load",
		.usage = "<path> <base_address> <size>",
	},
	{
		.name = "batching",
		.handler = &vdebug_set_batching,
		.mode = COMMAND_CONFIG,
		.help = "set the transaction batching no|wr|rd [0|1|2]",
		.usage = "<level>",
	},
	{
		.name = "polling",
		.handler = &vdebug_set_polling,
		.mode = COMMAND_CONFIG,
		.help = "set the polling pause, executing hardware cycles between min and max",
		.usage = "<min cycles> <max cycles>",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration vdebug_command[] = {
	{
		.name = "vdebug",
		.chain = vdebug_command_handlers,
		.mode = COMMAND_ANY,
		.help = "vdebug command group",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface vdebug_jtag_ops = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = vdebug_jtag_execute_queue,
};

struct adapter_driver vdebug_adapter_driver = {
	.name = "vdebug",
	.transports = jtag_only,
	.speed = vdebug_jtag_speed,
	.khz = vdebug_jtag_khz,
	.speed_div = vdebug_jtag_div,
	.commands = vdebug_command,
	.init = vdebug_init,
	.quit = vdebug_quit,
	.reset = vdebug_reset,
	.jtag_ops = &vdebug_jtag_ops,
};
