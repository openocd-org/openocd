// SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause)

/* Copyright 2020-2022 Cadence Design Systems, Inc. */

/*!
 * @file
 *
 * @brief the virtual debug interface provides a connection between a sw debugger
 * and the simulated, emulated core. The openOCD client connects via TCP sockets
 * with vdebug server and over DPI-based transactor with the emulation or simulation
 * The vdebug debug driver supports JTAG and DAP-level transports
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
#include "target/arm_adi_v5.h"
#include "helper/time_support.h"
#include "helper/replacements.h"
#include "helper/log.h"
#include "helper/list.h"

#define VD_VERSION 48
#define VD_BUFFER_LEN 4024
#define VD_CHEADER_LEN 24
#define VD_SHEADER_LEN 16

#define VD_MAX_MEMORIES 20
#define VD_POLL_INTERVAL 500
#define VD_SCALE_PSTOMS 1000000000

/**
 * @brief List of transactor types
 */
enum {
	VD_BFM_TPIU   = 0x0000,  /* transactor trace TPIU */
	VD_BFM_DAP6   = 0x0001,  /* transactor DAP ADI V6 */
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
	VD_CMD_REGWRITE   = 0x15,
	VD_CMD_REGREAD    = 0x16,
	VD_CMD_JTAGSHTAP  = 0x1a,
	VD_CMD_MEMOPEN    = 0x21,
	VD_CMD_MEMCLOSE   = 0x22,
	VD_CMD_MEMWRITE   = 0x23,
};

enum {
	VD_ASPACE_AP      = 0x01,
	VD_ASPACE_DP      = 0x02,
	VD_ASPACE_ID      = 0x03,
	VD_ASPACE_AB      = 0x04,
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
		uint8_t waddr[2];        /* 002; write pointer */
		uint8_t wbytes[2];       /* 004; data bytes */
		uint8_t rbytes[2];       /* 006; data bytes to read */
		uint8_t wwords[2];       /* 008; data words */
		uint8_t rwords[2];       /* 00a; data words to read */
		uint8_t rwdata[4];       /* 00c; read/write data */
		uint8_t offset[4];       /* 010; address offset */
		uint8_t offseth[2];      /* 014; address offset 47:32 */
		uint8_t wid[2];          /* 016; request id*/
	};
	uint8_t wd8[VD_BUFFER_LEN];  /* 018; */
	struct {                     /* VD_SHEADER_LEN written by server */
		uint8_t rid[2];          /* fd0: request id read */
		uint8_t awords[2];       /* fd2: actual data words read back */
		uint8_t status[4];       /* fd4; */
		uint8_t duttime[8];      /* fd8; */
	};
	uint8_t rd8[VD_BUFFER_LEN];  /* fe0: */
	uint8_t state[4];            /* 1f98; connection state */
	uint8_t count[4];            /* 1f9c; */
	uint8_t dummy[96];           /* 1fa0; 48+40B+8B; */
} __attribute__((packed));

struct vd_rdata {
	struct list_head lh;
	uint8_t *rdata;
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
	struct vd_rdata rdataq;
};

struct vd_jtag_hdr {
	uint64_t tlen:24;
	uint64_t post:3;
	uint64_t pre:3;
	uint64_t cmd:2;
	uint64_t wlen:16;
	uint64_t rlen:16;
};

struct vd_reg_hdr {
	uint64_t prot:3;
	uint64_t nonincr:1;
	uint64_t haddr:12;
	uint64_t tlen:11;
	uint64_t asize:3;
	uint64_t cmd:2;
	uint64_t addr:32;
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
	if (hsock < 0)
		rc = vdebug_socket_error();
#elif defined __CYGWIN__
	/* SO_RCVLOWAT unsupported on CYGWIN */
	hsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (hsock < 0)
		rc = errno;
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
		h_u16_to_be((uint8_t *)ainfo->ai_addr->sa_data, port);
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
	int offset = &pmem->rid[0] - &pmem->cmd;
	int to_receive = VD_SHEADER_LEN + le_to_h_u16(pmem->rbytes);
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
	int rc = send(hsock, (const char *)&pmem->cmd, VD_CHEADER_LEN + le_to_h_u16(pmem->wbytes), 0);
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

	int rc = le_to_h_u32(pmem->status);
	LOG_DEBUG_IO("wait_server: cmd %02" PRIx8 " done, sent %d, rcvd %d, status %d",
				 pmem->cmd, st, rd, rc);

	return rc;
}

static int vdebug_run_jtag_queue(int hsock, struct vd_shm *pm, unsigned int count)
{
	uint8_t  num_pre, num_post, tdi, tms;
	unsigned int num, anum, bytes, hwords, words;
	unsigned int req, waddr, rwords;
	int64_t ts, te;
	uint8_t *tdo;
	int rc;
	uint64_t jhdr;
	struct vd_rdata *rd;

	req = 0;                            /* beginning of request */
	waddr = 0;
	rwords = 0;
	h_u16_to_le(pm->wbytes, le_to_h_u16(pm->wwords) * vdc.buf_width);
	h_u16_to_le(pm->rbytes, le_to_h_u16(pm->rwords) * vdc.buf_width);
	ts = timeval_ms();
	rc = vdebug_wait_server(hsock, pm);
	while (!rc && (req < count)) {      /* loop over requests to read data and print out */
		jhdr = le_to_h_u64(&pm->wd8[waddr * 4]);
		words = jhdr >> 48;
		hwords = (jhdr >> 32) & 0xffff;
		anum = jhdr & 0xffffff;
		num_pre = (jhdr >> 27) & 0x7;
		num_post = (jhdr >> 24) & 0x7;
		if (num_post)
			num = anum - num_pre - num_post + 1;
		else
			num = anum - num_pre;
		bytes = (num + 7) / 8;
		vdc.trans_last = (req + 1) < count ? 0 : 1;
		vdc.trans_first = waddr ? 0 : 1;
		if (((jhdr >> 30) & 0x3) == 3) { /* cmd is read */
			if (!rwords) {
				rd = &vdc.rdataq;
				tdo = rd->rdata;
			} else {
				rd = list_first_entry(&vdc.rdataq.lh, struct vd_rdata, lh);
				tdo = rd->rdata;
				list_del(&rd->lh);
				free(rd);
			}
			for (unsigned int j = 0; j < bytes; j++) {
				tdo[j] = (pm->rd8[rwords * 8 + j] >> num_pre) | (pm->rd8[rwords * 8 + j + 1] << (8 - num_pre));
				LOG_DEBUG_IO("%04x D0[%02x]:%02x", le_to_h_u16(pm->wid) - count + req, j, tdo[j]);
			}
			rwords += words;           /* read data offset */
		} else {
			tdo = NULL;
		}
		waddr += sizeof(uint64_t) / 4; /* waddr past header */
		tdi = (pm->wd8[waddr * 4] >> num_pre) | (pm->wd8[waddr * 4 + 1] << (8 - num_pre));
		tms = (pm->wd8[waddr * 4 + 4] >> num_pre) | (pm->wd8[waddr * 4 + 4 + 1] << (8 - num_pre));
		LOG_DEBUG_IO("%04x L:%02d O:%05x @%03x DI:%02x MS:%02x DO:%02x",
			le_to_h_u16(pm->wid) - count + req, num, (vdc.trans_first << 14) | (vdc.trans_last << 15),
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
	h_u16_to_le(pm->offseth, 0);      /* reset buffer write address */
	h_u32_to_le(pm->offset, 0);
	h_u16_to_le(pm->rwords, 0);
	h_u16_to_le(pm->waddr, 0);
	assert(list_empty(&vdc.rdataq.lh));/* list should be empty after run queue */

	return rc;
}

static int vdebug_run_reg_queue(int hsock, struct vd_shm *pm, unsigned int count)
{
	unsigned int num, awidth, wwidth;
	unsigned int req, waddr, rwords;
	uint8_t aspace;
	uint32_t addr;
	int64_t ts, te;
	uint8_t *data;
	int rc;
	uint64_t rhdr;
	struct vd_rdata *rd;

	req = 0;                            /* beginning of request */
	waddr = 0;
	rwords = 0;
	h_u16_to_le(pm->wbytes, le_to_h_u16(pm->wwords) * vdc.buf_width);
	h_u16_to_le(pm->rbytes, le_to_h_u16(pm->rwords) * vdc.buf_width);
	ts = timeval_ms();
	rc = vdebug_wait_server(hsock, pm);
	while (!rc && (req < count)) {      /* loop over requests to read data and print out */
		rhdr = le_to_h_u64(&pm->wd8[waddr * 4]);
		addr = rhdr >> 32;              /* reconstruct data for a single request */
		num = (rhdr >> 16) & 0x7ff;
		aspace = rhdr & 0x3;
		awidth = (1 << ((rhdr >> 27) & 0x7));
		wwidth = (awidth + vdc.buf_width - 1) / vdc.buf_width;
		vdc.trans_last = (req + 1) < count ? 0 : 1;
		vdc.trans_first = waddr ? 0 : 1;
		if (((rhdr >> 30) & 0x3) == 2) { /* cmd is read */
			if (num) {
				if (!rwords) {
					rd = &vdc.rdataq;
					data = rd->rdata;
				} else {
					rd = list_first_entry(&vdc.rdataq.lh, struct vd_rdata, lh);
					data = rd->rdata;
					list_del(&rd->lh);
					free(rd);
				}
				for (unsigned int j = 0; j < num; j++)
					memcpy(&data[j * awidth], &pm->rd8[(rwords + j) * awidth], awidth);
			}
			LOG_DEBUG("read  %04x AS:%1x RG:%1x O:%05x @%03x D:%08x", le_to_h_u16(pm->wid) - count + req,
				aspace, addr << 2, (vdc.trans_first << 14) | (vdc.trans_last << 15), waddr,
				(num ? le_to_h_u32(&pm->rd8[rwords * 4]) : 0xdead));
			rwords += num * wwidth;
			waddr += sizeof(uint64_t) / 4; /* waddr past header */
		} else {
			LOG_DEBUG("write %04x AS:%1x RG:%1x O:%05x @%03x D:%08x", le_to_h_u16(pm->wid) - count + req,
				aspace, addr << 2, (vdc.trans_first << 14) | (vdc.trans_last << 15), waddr,
				le_to_h_u32(&pm->wd8[(waddr + num + 1) * 4]));
			waddr += sizeof(uint64_t) / 4 + (num * wwidth * awidth + 3) / 4;
		}
		req += 1;
	}

	if (rc) {
		LOG_ERROR("0x%x executing transaction", rc);
		rc = ERROR_FAIL;
	}

	te = timeval_ms();
	vdc.targ_time += (uint32_t)(te - ts);
	h_u16_to_le(pm->offseth, 0);      /* reset buffer write address */
	h_u32_to_le(pm->offset, 0);
	h_u16_to_le(pm->rwords, 0);
	h_u16_to_le(pm->waddr, 0);
	assert(list_empty(&vdc.rdataq.lh));/* list should be empty after run queue */

	return rc;
}

static int vdebug_open(int hsock, struct vd_shm *pm, const char *path,
						uint8_t type, uint32_t period_ps, uint32_t sig_mask)
{
	int rc = VD_ERR_NOT_OPEN;

	pm->cmd = VD_CMD_OPEN;
	h_u16_to_le(pm->wid, VD_VERSION);  /* client version */
	h_u16_to_le(pm->wbytes, 0);
	h_u16_to_le(pm->rbytes, 0);
	h_u16_to_le(pm->wwords, 0);
	h_u16_to_le(pm->rwords, 0);
	rc = vdebug_wait_server(hsock, pm);
	if (rc != 0) {                     /* communication problem */
		LOG_ERROR("0x%x connecting to server", rc);
	} else if (le_to_h_u16(pm->rid) < le_to_h_u16(pm->wid)) {
		LOG_ERROR("server version %d too old for the client %d", le_to_h_u16(pm->rid), le_to_h_u16(pm->wid));
		pm->cmd = VD_CMD_CLOSE;        /* let server close the connection */
		vdebug_wait_server(hsock, pm);
		rc = VD_ERR_VERSION;
	} else {
		pm->cmd = VD_CMD_CONNECT;
		pm->type = type;               /* BFM type to connect to */
		h_u32_to_le(pm->rwdata, sig_mask | VD_SIG_BUF | (VD_SIG_BUF << 16));
		h_u16_to_le(pm->wbytes, strlen(path) + 1);
		h_u16_to_le(pm->rbytes, 12);
		h_u16_to_le(pm->wid, 0);       /* reset wid for transaction ID */
		h_u16_to_le(pm->wwords, 0);
		h_u16_to_le(pm->rwords, 0);
		memcpy(pm->wd8, path, le_to_h_u16(pm->wbytes));
		rc = vdebug_wait_server(hsock, pm);
		vdc.sig_read = le_to_h_u32(pm->rwdata) >> 16;  /* signal read mask */
		vdc.sig_write = le_to_h_u32(pm->rwdata);     /* signal write mask */
		vdc.bfm_period = period_ps;
		vdc.buf_width = le_to_h_u32(&pm->rd8[0]) / 8;/* access width in bytes */
		vdc.addr_bits = le_to_h_u32(&pm->rd8[2 * 4]);    /* supported address bits */
	}

	if (rc) {
		LOG_ERROR("0x%x connecting to BFM %s", rc, path);
		return ERROR_FAIL;
	}

	INIT_LIST_HEAD(&vdc.rdataq.lh);
	LOG_DEBUG("%s type %0x, period %dps, buffer %dx%dB signals r%04xw%04x",
		path, type, vdc.bfm_period, VD_BUFFER_LEN / vdc.buf_width,
		vdc.buf_width, vdc.sig_read, vdc.sig_write);

	return ERROR_OK;
}

static int vdebug_close(int hsock, struct vd_shm *pm, uint8_t type)
{
	pm->cmd = VD_CMD_DISCONNECT;
	pm->type = type;              /* BFM type, here JTAG */
	h_u16_to_le(pm->wbytes, 0);
	h_u16_to_le(pm->rbytes, 0);
	h_u16_to_le(pm->wwords, 0);
	h_u16_to_le(pm->rwords, 0);
	vdebug_wait_server(hsock, pm);
	pm->cmd = VD_CMD_CLOSE;
	h_u16_to_le(pm->wid, VD_VERSION);    /* client version */
	h_u16_to_le(pm->wbytes, 0);
	h_u16_to_le(pm->rbytes, 0);
	h_u16_to_le(pm->wwords, 0);
	h_u16_to_le(pm->rwords, 0);
	vdebug_wait_server(hsock, pm);
	LOG_DEBUG("type %0x", type);

	return ERROR_OK;
}

static int vdebug_wait(int hsock, struct vd_shm *pm, uint32_t cycles)
{
	if (cycles) {
		pm->cmd = VD_CMD_WAIT;
		h_u16_to_le(pm->wbytes, 0);
		h_u16_to_le(pm->rbytes, 0);
		h_u32_to_le(pm->rwdata, cycles);  /* clock sycles to wait */
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
	h_u16_to_le(pm->wbytes, 0);
	h_u16_to_le(pm->rbytes, 0);
	h_u32_to_le(pm->rwdata, (write_mask << 16) | (value & 0xffff)); /* mask and value of signals to set */
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
	h_u16_to_le(pm->wbytes, 0);
	h_u16_to_le(pm->rbytes, 0);
	h_u32_to_le(pm->rwdata, value);  /* divider value */
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
	vdc.trans_last = f_last || (vdc.trans_batch == VD_BATCH_NO);
	if (vdc.trans_first)
		waddr = 0;             /* reset buffer offset */
	else
		waddr = le_to_h_u32(pm->offseth);   /* continue from the previous transaction */
	if (num_post)          /* actual number of bits to shift */
		anum = num + num_pre + num_post - 1;
	else
		anum = num + num_pre;
	hwords = (anum + 4 * vdc.buf_width - 1) / (4 * vdc.buf_width); /* in 4B TDI/TMS words */
	words = (hwords + 1) / 2;    /* in 8B TDO words to read */
	bytes = (num + 7) / 8;       /* data only portion in bytes */
	/* buffer overflow check and flush */
	if (4 * waddr + sizeof(uint64_t) + 8 * hwords + 64 > VD_BUFFER_LEN) {
		vdc.trans_last = 1;        /* force flush within 64B of buffer end */
	} else if (4 * waddr + sizeof(uint64_t) + 8 * hwords > VD_BUFFER_LEN) {
		/* this req does not fit, discard it */
		LOG_ERROR("%04x L:%02d O:%05x @%04x too many bits to shift",
			le_to_h_u16(pm->wid), anum, (vdc.trans_first << 14) | (vdc.trans_last << 15), waddr);
		rc = ERROR_FAIL;
	}

	if (!rc && anum) {
		uint16_t i, j;       /* portability requires to use bit operations for 8B JTAG header */
		uint64_t jhdr = (tdo ? ((uint64_t)(words) << 48) : 0) + ((uint64_t)(hwords) << 32) +
			((tdo ? 3UL : 1UL) << 30) + (num_pre << 27) + (num_post << 24) + anum;
		h_u64_to_le(&pm->wd8[4 * waddr], jhdr);

		h_u16_to_le(pm->wid, le_to_h_u16(pm->wid) + 1);    /* transaction ID */
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
			struct vd_rdata *rd;
			if (le_to_h_u16(pm->rwords) == 0) {
				rd = &vdc.rdataq;
			} else {
				rd = calloc(1, sizeof(struct vd_rdata));
				if (!rd)                   /* check allocation for 24B */
					return ERROR_FAIL;
				list_add_tail(&rd->lh, &vdc.rdataq.lh);
			}
			rd->rdata = tdo;
			h_u16_to_le(pm->rwords, le_to_h_u16(pm->rwords) + words);/* keep track of the words to read */
		}
		h_u16_to_le(pm->wwords, waddr / 2 + hwords); /* payload size *2 to include both TDI and TMS data */
		h_u16_to_le(pm->waddr, le_to_h_u16(pm->waddr) + 1);
	}

	if (!waddr)                        /* flush issued, but buffer empty */
		;
	else if (!vdc.trans_last)          /* buffered request */
		h_u16_to_le(pm->offseth, waddr + hwords * 2);  /* offset for next transaction, must be even */
	else                               /* execute batch of requests */
		rc = vdebug_run_jtag_queue(hsock, pm, le_to_h_u16(pm->waddr));
	vdc.trans_first = vdc.trans_last; /* flush forces trans_first flag */

	return rc;
}

static int vdebug_reg_write(int hsock, struct vd_shm *pm, const uint32_t reg,
							const uint32_t data, uint8_t aspace, uint8_t f_last)
{
	uint32_t waddr;
	int rc = ERROR_OK;

	pm->cmd = VD_CMD_REGWRITE;
	vdc.trans_last = f_last || (vdc.trans_batch == VD_BATCH_NO);
	if (vdc.trans_first)
		waddr = 0;             /* reset buffer offset */
	else
		waddr = le_to_h_u16(pm->offseth);   /* continue from the previous transaction */

	if (4 * waddr + 2 * sizeof(uint64_t) + 4 > VD_BUFFER_LEN)
		vdc.trans_last = 1;    /* force flush, no room for next request */

	uint64_t rhdr = ((uint64_t)reg << 32) + (1UL << 30) + (2UL << 27) + (1UL << 16) + aspace;
	h_u64_to_le(&pm->wd8[4 * waddr], rhdr);
	h_u32_to_le(&pm->wd8[4 * (waddr + 2)], data);
	h_u16_to_le(pm->wid, le_to_h_u16(pm->wid) + 1);
	h_u16_to_le(pm->wwords, waddr + 3);
	h_u16_to_le(pm->waddr, le_to_h_u16(pm->waddr) + 1);
	if (!vdc.trans_last)       /* buffered request */
		h_u16_to_le(pm->offseth, waddr + 3);
	else
		rc = vdebug_run_reg_queue(hsock, pm, le_to_h_u16(pm->waddr));
	vdc.trans_first = vdc.trans_last; /* flush forces trans_first flag */

	return rc;
}

static int vdebug_reg_read(int hsock, struct vd_shm *pm, const uint32_t reg,
							uint32_t *data, uint8_t aspace, uint8_t f_last)
{
	uint32_t waddr;
	int rc = ERROR_OK;

	pm->cmd = VD_CMD_REGREAD;
	vdc.trans_last = f_last || (vdc.trans_batch == VD_BATCH_NO);
	if (vdc.trans_first)
		waddr = 0;             /* reset buffer offset */
	else
		waddr = le_to_h_u16(pm->offseth);   /* continue from the previous transaction */

	if (4 * waddr + 2 * sizeof(uint64_t) + 4 > VD_BUFFER_LEN)
		vdc.trans_last = 1;    /* force flush, no room for next request */

	uint64_t rhdr = ((uint64_t)reg << 32) + (2UL << 30) + (2UL << 27) + ((data ? 1UL : 0UL) << 16) + aspace;
	h_u64_to_le(&pm->wd8[4 * waddr], rhdr);
	h_u16_to_le(pm->wid, le_to_h_u16(pm->wid) + 1);
	if (data) {
		struct vd_rdata *rd;
		if (le_to_h_u16(pm->rwords) == 0) {
			rd = &vdc.rdataq;
		} else {
			rd = calloc(1, sizeof(struct vd_rdata));
			if (!rd)                   /* check allocation for 24B */
				return ERROR_FAIL;
			list_add_tail(&rd->lh, &vdc.rdataq.lh);
		}
		rd->rdata = (uint8_t *)data;
		h_u16_to_le(pm->rwords, le_to_h_u16(pm->rwords) + 1);
	}
	h_u16_to_le(pm->wwords, waddr + 2);
	h_u16_to_le(pm->waddr, le_to_h_u16(pm->waddr) + 1);
	if (!vdc.trans_last)       /* buffered request */
		h_u16_to_le(pm->offseth, waddr + 2);
	else
		rc = vdebug_run_reg_queue(hsock, pm, le_to_h_u16(pm->waddr));
	vdc.trans_first = vdc.trans_last; /* flush forces trans_first flag */

	return rc;
}

static int vdebug_mem_open(int hsock, struct vd_shm *pm, const char *path, uint8_t ndx)
{
	int rc;

	if (!path)
		return ERROR_OK;

	pm->cmd = VD_CMD_MEMOPEN;
	h_u16_to_le(pm->wbytes, strlen(path) + 1);   /* includes terminating 0 */
	h_u16_to_le(pm->rbytes, 8);
	h_u16_to_le(pm->wwords, 0);
	h_u16_to_le(pm->rwords, 0);
	memcpy(pm->wd8, path, le_to_h_u16(pm->wbytes));
	rc = vdebug_wait_server(hsock, pm);
	if (rc) {
		LOG_ERROR("0x%x opening memory %s", rc, path);
	} else if (ndx != pm->rd8[2]) {
		LOG_WARNING("Invalid memory index %" PRIu16 " returned. Direct memory access disabled", pm->rd8[2]);
	} else {
		vdc.mem_width[ndx] = le_to_h_u16(&pm->rd8[0]) / 8;   /* memory width in bytes */
		vdc.mem_depth[ndx] = le_to_h_u32(&pm->rd8[4]);       /* memory depth in words */
		LOG_DEBUG("%" PRIx8 ": %s memory %" PRIu32 "x%" PRIu32 "B, buffer %" PRIu32 "x%" PRIu32 "B", ndx, path,
			vdc.mem_depth[ndx], vdc.mem_width[ndx], VD_BUFFER_LEN / vdc.mem_width[ndx], vdc.mem_width[ndx]);
	}

	return ERROR_OK;
}

static void vdebug_mem_close(int hsock, struct vd_shm *pm, uint8_t ndx)
{
	pm->cmd = VD_CMD_MEMCLOSE;
	h_u32_to_le(pm->rwdata, ndx);        /* which memory */
	h_u16_to_le(pm->wbytes, 0);
	h_u16_to_le(pm->rbytes, 0);
	h_u16_to_le(pm->wwords, 0);
	h_u16_to_le(pm->rwords, 0);
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
		LOG_ERROR("cannot allocate %zu bytes", sizeof(struct vd_shm));
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
	uint32_t sig_mask = VD_SIG_RESET;
	if (transport_is_jtag())
		sig_mask |= VD_SIG_TRST | VD_SIG_TCKDIV;

	int rc = vdebug_open(vdc.hsocket, pbuf, vdc.bfm_path, vdc.bfm_type, vdc.bfm_period, sig_mask);
	if (rc != 0) {
		LOG_ERROR("0x%x cannot connect to %s", rc, vdc.bfm_path);
		close_socket(vdc.hsocket);
		vdc.hsocket = 0;
		free(pbuf);
		pbuf = NULL;
	} else {
		for (uint8_t i = 0; i < vdc.mem_ndx; i++) {
			rc = vdebug_mem_open(vdc.hsocket, pbuf, vdc.mem_path[i], i);
			if (rc != 0)
				LOG_ERROR("0x%x cannot connect to %s", rc, vdc.mem_path[i]);
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
	LOG_DEBUG_IO("tms  len:%d tms:%x", num, *tms);

	return vdebug_jtag_shift_tap(vdc.hsocket, pbuf, num, *tms, 0, NULL, 0, 0, NULL, f_flush);
}

static int vdebug_jtag_path_move(struct pathmove_command *cmd, uint8_t f_flush)
{
	uint8_t tms[DIV_ROUND_UP(cmd->num_states, 8)];
	LOG_DEBUG_IO("path num states %u", cmd->num_states);

	memset(tms, 0, DIV_ROUND_UP(cmd->num_states, 8));

	for (unsigned int i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			buf_set_u32(tms, i, 1, 1);
		tap_set_state(cmd->path[i]);
	}

	return vdebug_jtag_tms_seq(tms, cmd->num_states, f_flush);
}

static int vdebug_jtag_tlr(enum tap_state state, uint8_t f_flush)
{
	int rc = ERROR_OK;

	enum tap_state cur = tap_get_state();
	uint8_t tms_pre = tap_get_tms_path(cur, state);
	uint8_t num_pre = tap_get_tms_path_len(cur, state);
	LOG_DEBUG_IO("tlr  from %x to %x", cur, state);
	if (cur != state) {
		rc = vdebug_jtag_shift_tap(vdc.hsocket, pbuf, num_pre, tms_pre, 0, NULL, 0, 0, NULL, f_flush);
		tap_set_state(state);
	}

	return rc;
}

static int vdebug_jtag_scan(struct scan_command *cmd, uint8_t f_flush)
{
	int rc = ERROR_OK;

	enum tap_state cur = tap_get_state();
	uint8_t state = cmd->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT;
	uint8_t tms_pre = tap_get_tms_path(cur, state);
	uint8_t num_pre = tap_get_tms_path_len(cur, state);
	uint8_t tms_post = tap_get_tms_path(state, cmd->end_state);
	uint8_t num_post = tap_get_tms_path_len(state, cmd->end_state);
	const unsigned int num_bits = jtag_scan_size(cmd);
	LOG_DEBUG_IO("scan len:%u fields:%u ir/!dr:%d state cur:%x end:%x",
			  num_bits, cmd->num_fields, cmd->ir_scan, cur, cmd->end_state);
	for (unsigned int i = 0; i < cmd->num_fields; i++) {
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

static int vdebug_jtag_runtest(unsigned int num_cycles, enum tap_state state, uint8_t f_flush)
{
	enum tap_state cur = tap_get_state();
	uint8_t tms_pre = tap_get_tms_path(cur, state);
	uint8_t num_pre = tap_get_tms_path_len(cur, state);
	LOG_DEBUG_IO("idle len:%u state cur:%x end:%x", num_cycles, cur, state);
	int rc = vdebug_jtag_shift_tap(vdc.hsocket, pbuf, num_pre, tms_pre, num_cycles, NULL, 0, 0, NULL, f_flush);
	if (cur != state)
		tap_set_state(state);

	return rc;
}

static int vdebug_jtag_stableclocks(unsigned int num_cycles, uint8_t f_flush)
{
	LOG_DEBUG("stab len:%u state cur:%x", num_cycles, tap_get_state());

	return vdebug_jtag_shift_tap(vdc.hsocket, pbuf, 0, 0, num_cycles, NULL, 0, 0, NULL, f_flush);
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

static int vdebug_jtag_execute_queue(struct jtag_command *cmd_queue)
{
	int rc = ERROR_OK;

	for (struct jtag_command *cmd = cmd_queue; rc == ERROR_OK && cmd; cmd = cmd->next) {
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

static int vdebug_dap_bankselect(struct adiv5_ap *ap, unsigned int reg)
{
	int rc = ERROR_OK;
	uint64_t sel;

	if (is_adiv6(ap->dap)) {
		sel = ap->ap_num | (reg & 0x00000FF0);
		if (sel != (ap->dap->select & ~0xfull)) {
			sel |= ap->dap->select & DP_SELECT_DPBANK;
			if (ap->dap->asize > 32)
				sel |= (DP_SELECT1 >> 4) & DP_SELECT_DPBANK;
			ap->dap->select = sel;
			ap->dap->select_valid = true;
			rc = vdebug_reg_write(vdc.hsocket, pbuf, DP_SELECT >> 2, (uint32_t)sel, VD_ASPACE_DP, 0);
			if (rc == ERROR_OK) {
				ap->dap->select_valid = true;
				if (ap->dap->asize > 32)
					rc = vdebug_reg_write(vdc.hsocket, pbuf, (DP_SELECT1 & DP_SELECT_DPBANK) >> 2,
					(uint32_t)(sel >> 32), VD_ASPACE_DP, 0);
				if (rc == ERROR_OK)
					ap->dap->select1_valid = true;
			}
		}
	} else {    /* ADIv5 */
		sel = (ap->ap_num << 24) | (reg & ADIV5_DP_SELECT_APBANK);
		if (sel != ap->dap->select) {
			ap->dap->select = sel;
			rc = vdebug_reg_write(vdc.hsocket, pbuf, DP_SELECT >> 2, (uint32_t)sel, VD_ASPACE_DP, 0);
			if (rc == ERROR_OK)
				ap->dap->select_valid = true;
		}
	}
	return rc;
}

static int vdebug_dap_connect(struct adiv5_dap *dap)
{
	return dap_dp_init(dap);
}

static int vdebug_dap_send_sequence(struct adiv5_dap *dap, enum swd_special_seq seq)
{
	return ERROR_OK;
}

static int vdebug_dap_queue_dp_read(struct adiv5_dap *dap, unsigned int reg, uint32_t *data)
{
	if (reg != DP_SELECT && reg != DP_RDBUFF
		&& (!dap->select_valid || ((reg >> 4) & DP_SELECT_DPBANK) != (dap->select & DP_SELECT_DPBANK))) {
		dap->select = (dap->select & ~DP_SELECT_DPBANK) | ((reg >> 4) & DP_SELECT_DPBANK);
		vdebug_reg_write(vdc.hsocket, pbuf, DP_SELECT >> 2, dap->select, VD_ASPACE_DP, 0);
		dap->select_valid = true;
	}
	return vdebug_reg_read(vdc.hsocket, pbuf, (reg & DP_SELECT_DPBANK) >> 2, data, VD_ASPACE_DP, 0);
}

static int vdebug_dap_queue_dp_write(struct adiv5_dap *dap, unsigned int reg, uint32_t data)
{
	if (reg != DP_SELECT && reg != DP_RDBUFF
		&& (!dap->select_valid || ((reg >> 4) & DP_SELECT_DPBANK) != (dap->select & DP_SELECT_DPBANK))) {
		dap->select = (dap->select & ~DP_SELECT_DPBANK) | ((reg >> 4) & DP_SELECT_DPBANK);
		vdebug_reg_write(vdc.hsocket, pbuf, DP_SELECT >> 2, dap->select, VD_ASPACE_DP, 0);
		dap->select_valid = true;
	}
	return vdebug_reg_write(vdc.hsocket, pbuf, (reg & DP_SELECT_DPBANK) >> 2, data, VD_ASPACE_DP, 0);
}

static int vdebug_dap_queue_ap_read(struct adiv5_ap *ap, unsigned int reg, uint32_t *data)
{
	vdebug_dap_bankselect(ap, reg);

	vdebug_reg_read(vdc.hsocket, pbuf, (reg & DP_SELECT_DPBANK) >> 2, NULL, VD_ASPACE_AP, 0);

	return vdebug_reg_read(vdc.hsocket, pbuf, DP_RDBUFF >> 2, data, VD_ASPACE_DP, 0);
}

static int vdebug_dap_queue_ap_write(struct adiv5_ap *ap, unsigned int reg, uint32_t data)
{
	vdebug_dap_bankselect(ap, reg);
	return vdebug_reg_write(vdc.hsocket, pbuf, (reg & DP_SELECT_DPBANK) >> 2, data, VD_ASPACE_AP, 0);
}

static int vdebug_dap_queue_ap_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	return vdebug_reg_write(vdc.hsocket, pbuf, 0, 0x1, VD_ASPACE_AB, 0);
}

static int vdebug_dap_run(struct adiv5_dap *dap)
{
	if (le_to_h_u16(pbuf->waddr))
		return vdebug_run_reg_queue(vdc.hsocket, pbuf, le_to_h_u16(pbuf->waddr));

	return ERROR_OK;
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
	if (transport_is_dapdirect_swd())
		vdc.bfm_type = strstr(vdc.bfm_path, "dap6") ? VD_BFM_DAP6 : VD_BFM_SWDP;
	else
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
		.mode = COMMAND_CONFIG,
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

static const struct dap_ops vdebug_dap_ops = {
	.connect = vdebug_dap_connect,
	.send_sequence = vdebug_dap_send_sequence,
	.queue_dp_read = vdebug_dap_queue_dp_read,
	.queue_dp_write = vdebug_dap_queue_dp_write,
	.queue_ap_read = vdebug_dap_queue_ap_read,
	.queue_ap_write = vdebug_dap_queue_ap_write,
	.queue_ap_abort = vdebug_dap_queue_ap_abort,
	.run = vdebug_dap_run,
	.sync = NULL, /* optional */
	.quit = NULL, /* optional */
};

static const char *const vdebug_transports[] = { "jtag", "dapdirect_swd", NULL };

struct adapter_driver vdebug_adapter_driver = {
	.name = "vdebug",
	.transports = vdebug_transports,
	.speed = vdebug_jtag_speed,
	.khz = vdebug_jtag_khz,
	.speed_div = vdebug_jtag_div,
	.commands = vdebug_command,
	.init = vdebug_init,
	.quit = vdebug_quit,
	.reset = vdebug_reset,
	.jtag_ops = &vdebug_jtag_ops,
	.dap_swd_ops = &vdebug_dap_ops,
};
