/******************************************************************************
 * RTL8188FU RX PHY statistics — per-packet descriptor capture + BB scan
 *
 * RTL8188FU is 1T1R (SISO). rOFDM1_CSI1/2 (0xd10/0xd18) are second-path
 * registers and always return 0 on this chip. Per-subcarrier CSI coefficients
 * are not exposed by this hardware — channel estimation is done internally.
 *
 * What this patch captures:
 *   Per-packet PHY descriptor (phy_status_rpt_8192cd), DMA'd by hardware:
 *     stream_rxevm[0]   → maps to rOFDM_RxEVMCSI (0xdd8), RX EVM magnitude
 *     path_rxsnr[0]     → SNR in 0.5 dB units, path A
 *     path_agc[0].gain  → 7-bit raw AGC gain (= RSSI proxy in monitor mode)
 *     noise_power_db_*  → noise floor estimate
 *     sig_evm, ch_corr  → signal EVM and channel correlation
 *   Processed fields from phydm_phy_sts_n_parsing():
 *     rx_pwdb_all, cfo_short/tail, rx_mimo_evm_dbm, rx_snr
 *
 * Note on 0xdd8 (rOFDM_RxEVMCSI): this register's stddev over time varies
 * with channel perturbation from human presence/motion. This observation is
 * empirical (tested on one device, one environment) — not derived from a
 * Realtek datasheet. Use with caution outside that context.
 *
 * procfs: /proc/net/rtl8188fu/<iface>/rx_phy_stats
 *         /proc/net/rtl8188fu/<iface>/bb_scan
 *         /proc/net/rtl8188fu/<iface>/c2h_log
 *****************************************************************************/

#include "mp_precomp.h"
#include "phydm_precomp.h"
#include "phydm_csi.h"

/* Last captured sample — updated per packet, read by procfs. */
static struct {
	/* descriptor raw fields */
	u8  csi0;		/* stream_csi[0] */
	u8  evm0;		/* stream_rxevm[0] as u8 */
	u8  snr0;		/* path_rxsnr[0] */
	u8  target_csi0;	/* stream_target_csi[0] */
	u8  noise_db;		/* noise_power_db_msb */
	u8  noise_lsb;		/* noise_power_db_lsb */
	s8  sig_evm;		/* sig_evm */
	u8  ch_corr0;		/* ch_corr[0] */
	u8  agc_gain;		/* path_agc[0].gain (7-bit raw AGC) */
	u8  agc_trsw;		/* path_agc[0].trsw */
	u8  pcts0;		/* pcts_mask[0] */
	u8  rxsc;		/* rxsc (2-bit subcarrier position) */
	u8  sgi_en;		/* sgi_en (short guard interval) */
	u8  ant_sel;		/* ant_sel */
	/* processed fields from phy_info (populated by phydm_phy_sts_n_parsing) */
	u8  pwdb_all;		/* rx_pwdb_all */
	s16 cfo_short0;		/* cfo_short[0] (kHz) */
	s16 cfo_tail0;		/* cfo_tail[0] (kHz) */
	u8  evm_dbm0;		/* rx_mimo_evm_dbm[0] */
	s8  rx_snr0;		/* rx_snr[0] */
	u8  csi_valid;
} csi_last;

void phydm_csi_query(struct dm_struct *dm,
		     struct phydm_phyinfo_struct *phy_info,
		     u8 *phy_status_inf)
{
	struct phy_status_rpt_8192cd *phy_sts;

	if (!dm || !phy_info || !phy_status_inf)
		return;

	phy_sts = (struct phy_status_rpt_8192cd *)phy_status_inf;

	/* --- descriptor raw fields --- */
	csi_last.csi0        = phy_sts->stream_csi[0];
	csi_last.evm0        = (u8)phy_sts->stream_rxevm[0];
	csi_last.snr0        = phy_sts->path_rxsnr[0];
	csi_last.target_csi0 = phy_sts->stream_target_csi[0];
	csi_last.noise_db    = phy_sts->noise_power_db_msb;
	csi_last.noise_lsb   = phy_sts->noise_power_db_lsb;
	csi_last.sig_evm     = phy_sts->sig_evm;
	csi_last.ch_corr0    = phy_sts->ch_corr[0];
	csi_last.agc_gain    = phy_sts->path_agc[0].gain;
	csi_last.agc_trsw    = phy_sts->path_agc[0].trsw;
	csi_last.pcts0       = phy_sts->pcts_mask[0];
	csi_last.rxsc        = phy_sts->rxsc;
	csi_last.sgi_en      = phy_sts->sgi_en;
	csi_last.ant_sel     = phy_sts->ant_sel;
	csi_last.csi_valid   = csi_last.snr0 ? 1 : 0;

	/* --- processed fields already in phy_info --- */
	csi_last.pwdb_all   = phy_info->rx_pwdb_all;
	csi_last.cfo_short0 = phy_info->cfo_short[0];
	csi_last.cfo_tail0  = phy_info->cfo_tail[0];
	csi_last.evm_dbm0   = phy_info->rx_mimo_evm_dbm[0];
	csi_last.rx_snr0    = phy_info->rx_snr[0];

	/* --- copy into phy_info for callers --- */
	phy_info->csi1_raw         = csi_last.csi0;
	phy_info->csi2_raw         = csi_last.evm0;
	phy_info->csi_evm_raw      = csi_last.snr0;
	phy_info->csi_valid        = csi_last.csi_valid;
	phy_info->phy_target_csi_a = csi_last.target_csi0;
	phy_info->phy_noise_db     = csi_last.noise_db;
	phy_info->phy_sig_evm      = csi_last.sig_evm;
	phy_info->phy_ch_corr_a    = csi_last.ch_corr0;

	PHYDM_DBG(dm, DBG_RSSI_MNTR,
		  "[CSI] csi0=%02x evm0=%02x snr0=%02x tgt=%02x noise=%02x sig_evm=%d corr=%02x valid=%d\n",
		  csi_last.csi0, csi_last.evm0, csi_last.snr0,
		  csi_last.target_csi0, csi_last.noise_db,
		  csi_last.sig_evm, csi_last.ch_corr0, csi_last.csi_valid);

	phydm_bb_scan_trigger(dm);
}

/* ---- BB register brute-force scan (workqueue, per-packet) ------------ */

/*
 * Covers 0xC00–0xEFF (3 × 256 bytes = 192 DWORDs, contiguous):
 *   0xC00–0xCFF: OFDM FA counters, signal quality (previously unscanned)
 *   0xD00–0xDFF: equalizer, CFO, EVM, AGC reports (original range)
 *   0xE00–0xEFF: unknown — chip-dependent (previously unscanned)
 */
#define BB_SCAN_BASE  0xC00
#define BB_SCAN_REGS  192  /* 0xC00–0xEFF: 768 bytes / 4 bytes = 192 DWORDs */

static struct {
	u32  cur[BB_SCAN_REGS];
	u32  prev[BB_SCAN_REGS];
	u8   changed[BB_SCAN_REGS]; /* 1 = differed from previous scan */
	u8   valid;
} bb_scan;

static struct work_struct  bb_scan_work;
static struct dm_struct   *bb_scan_dm;
static int                 bb_scan_initialized;

static void phydm_bb_scan_worker(struct work_struct *work)
{
	int i;

	if (!bb_scan_dm)
		return;

	for (i = 0; i < BB_SCAN_REGS; i++) {
		u32 val = odm_get_bb_reg(bb_scan_dm,
					 BB_SCAN_BASE + i * 4, MASKDWORD);
		bb_scan.prev[i]    = bb_scan.cur[i];
		bb_scan.cur[i]     = val;
		bb_scan.changed[i] = (val != bb_scan.prev[i]) ? 1 : 0;
	}
	bb_scan.valid = 1;
}

void phydm_bb_scan_trigger(struct dm_struct *dm)
{
	if (!bb_scan_initialized) {
		INIT_WORK(&bb_scan_work, phydm_bb_scan_worker);
		bb_scan_initialized = 1;
	}
	bb_scan_dm = dm;
	schedule_work(&bb_scan_work);
}

#ifdef CONFIG_PROC_DEBUG
void phydm_bb_scan_proc_show(void *m)
{
	struct seq_file *s = (struct seq_file *)m;
	int i, any;

	if (!s)
		return;

	if (!bb_scan.valid) {
		seq_puts(s, "BB_SCAN: no data yet\n");
		return;
	}

	seq_puts(s, "CHANGED:\n");
	any = 0;
	for (i = 0; i < BB_SCAN_REGS; i++) {
		if (bb_scan.changed[i]) {
			seq_printf(s, "  %03x: %08x -> %08x\n",
				   BB_SCAN_BASE + i * 4,
				   bb_scan.prev[i], bb_scan.cur[i]);
			any = 1;
		}
	}
	if (!any)
		seq_puts(s, "  (none)\n");

	seq_puts(s, "NONZERO:\n");
	any = 0;
	for (i = 0; i < BB_SCAN_REGS; i++) {
		if (bb_scan.cur[i]) {
			seq_printf(s, "  %03x: %08x%s\n",
				   BB_SCAN_BASE + i * 4,
				   bb_scan.cur[i],
				   bb_scan.changed[i] ? " *" : "");
			any = 1;
		}
	}
	if (!any)
		seq_puts(s, "  (none)\n");
}
#endif /* CONFIG_PROC_DEBUG */

/* ---- C2H packet ring buffer ------------------------------------------ */

#define C2H_LOG_MAX      16
#define C2H_PAYLOAD_MAX  16

static struct {
	u8  id;
	u8  seq;
	u8  plen;
	u8  payload[C2H_PAYLOAD_MAX];
} c2h_log[C2H_LOG_MAX];

static unsigned int c2h_log_head;  /* next write slot */
static unsigned int c2h_log_count; /* entries filled (capped at C2H_LOG_MAX) */

void phydm_c2h_log_entry(u8 id, u8 seq, u8 plen, u8 *payload)
{
	unsigned int slot = c2h_log_head;
	u8 copy = plen < C2H_PAYLOAD_MAX ? plen : C2H_PAYLOAD_MAX;

	c2h_log[slot].id   = id;
	c2h_log[slot].seq  = seq;
	c2h_log[slot].plen = plen;
	memcpy(c2h_log[slot].payload, payload, copy);

	c2h_log_head = (slot + 1) % C2H_LOG_MAX;
	if (c2h_log_count < C2H_LOG_MAX)
		c2h_log_count++;
}

#ifdef CONFIG_PROC_DEBUG
void phydm_c2h_log_proc_show(void *m)
{
	struct seq_file *s = (struct seq_file *)m;
	unsigned int i, start, count;

	if (!s)
		return;

	count = c2h_log_count;
	/* oldest entry is at (head - count) mod MAX */
	start = (c2h_log_head + C2H_LOG_MAX - count) % C2H_LOG_MAX;

	if (count == 0) {
		seq_puts(s, "C2H_LOG: empty\n");
		return;
	}

	for (i = 0; i < count; i++) {
		unsigned int idx = (start + i) % C2H_LOG_MAX;
		unsigned int j;
		u8 show = c2h_log[idx].plen < C2H_PAYLOAD_MAX
			  ? c2h_log[idx].plen : C2H_PAYLOAD_MAX;

		seq_printf(s, "C2H id=%02x seq=%02x plen=%02x payload=",
			   c2h_log[idx].id, c2h_log[idx].seq, c2h_log[idx].plen);
		for (j = 0; j < show; j++)
			seq_printf(s, "%02x", c2h_log[idx].payload[j]);
		seq_putc(s, '\n');
	}
}
#endif /* CONFIG_PROC_DEBUG */

/* ---- CSI procfs output ----------------------------------------------- */

#ifdef CONFIG_PROC_DEBUG
/*
 * phydm_rx_phy_stats_proc_show - dump all captured PHY fields to procfs
 *
 * Read via: cat /proc/net/rtl8188fu/wlan1/rx_phy_stats
 *
 * RAW_DESC fields come directly from phy_status_rpt_8192cd bytes.
 * SIGNAL fields are processed by phydm_phy_sts_n_parsing before capture.
 * BBP_* fields are live register reads (process context, safe to sleep).
 */
void phydm_rx_phy_stats_proc_show(struct dm_struct *dm, void *m)
{
	struct seq_file *s = (struct seq_file *)m;
	u32 r_intfdet, r_pnstate, r_agcrpt, r_shortcfo, r_longcfo, r_tailcfo;
	u32 r_bwrpt, r_phycnt1, r_phycnt2, r_phycnt3;

	if (!s || !dm)
		return;

	/* Live BBP register snapshot */
	r_intfdet  = odm_get_bb_reg(dm, 0xd3c, MASKDWORD);
	r_pnstate  = odm_get_bb_reg(dm, 0xd50, MASKDWORD);
	r_agcrpt   = odm_get_bb_reg(dm, 0xdd0, MASKDWORD);
	r_shortcfo = odm_get_bb_reg(dm, 0xdac, MASKDWORD);
	r_longcfo  = odm_get_bb_reg(dm, 0xdb4, MASKDWORD);
	r_tailcfo  = odm_get_bb_reg(dm, 0xdbc, MASKDWORD);
	r_bwrpt    = odm_get_bb_reg(dm, 0xdcc, MASKDWORD);
	r_phycnt1  = odm_get_bb_reg(dm, 0xda0, MASKDWORD);
	r_phycnt2  = odm_get_bb_reg(dm, 0xda4, MASKDWORD);
	r_phycnt3  = odm_get_bb_reg(dm, 0xda8, MASKDWORD);

	seq_printf(s,
		   "RAW_DESC:"
		   " CSI0=%02x"
		   " EVM0=%02x"
		   " SNR0=%02x"
		   " TCSI0=%02x"
		   " NOISE=%02x"
		   " NOISE_LSB=%02x"
		   " SIGEVM=%d"
		   " CORR0=%02x\n"
		   "AGC:"
		   " GAIN=%u"
		   " TRSW=%u"
		   " PCTS=%02x"
		   " RXSC=%u"
		   " SGI=%u"
		   " ANT=%u\n"
		   "SIGNAL:"
		   " PWDB=%u"
		   " CFO_S=%d"
		   " CFO_T=%d"
		   " EVMDBM=%u"
		   " RX_SNR=%d\n"
		   "CSI_VALID: %d\n"
		   "BBP_CFO: SHORT=%08x LONG=%08x TAIL=%08x\n"
		   "BBP_CHAN: INTFDET=%08x PNSTATE=%08x BWRPT=%08x\n"
		   "BBP_AGC: AGCRPT=%08x\n"
		   "BBP_CNT: PHY1=%08x PHY2=%08x PHY3=%08x\n",
		   csi_last.csi0,
		   csi_last.evm0,
		   csi_last.snr0,
		   csi_last.target_csi0,
		   csi_last.noise_db,
		   csi_last.noise_lsb,
		   csi_last.sig_evm,
		   csi_last.ch_corr0,
		   csi_last.agc_gain,
		   csi_last.agc_trsw,
		   csi_last.pcts0,
		   csi_last.rxsc,
		   csi_last.sgi_en,
		   csi_last.ant_sel,
		   csi_last.pwdb_all,
		   csi_last.cfo_short0,
		   csi_last.cfo_tail0,
		   csi_last.evm_dbm0,
		   csi_last.rx_snr0,
		   csi_last.csi_valid,
		   r_shortcfo, r_longcfo, r_tailcfo,
		   r_intfdet, r_pnstate, r_bwrpt,
		   r_agcrpt,
		   r_phycnt1, r_phycnt2, r_phycnt3);
}
#endif /* CONFIG_PROC_DEBUG */
