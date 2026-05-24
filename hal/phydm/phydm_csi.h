/******************************************************************************
 * RTL8188FU RX PHY statistics — procfs interface declarations
 *
 * RTL8188FU is 1T1R (SISO). rOFDM1_CSI1/2 (0xd10/0xd18) are second-path
 * registers and always return 0. Per-subcarrier CSI is not available.
 *
 * procfs entries exposed:
 *   rx_phy_stats — per-packet PHY descriptor fields (GAIN, PWDB, EVM, SNR,
 *                  NOISE, CSI_VALID, BBP_CFO, BBP_CHAN, BBP_AGC, BBP_CNT)
 *   bb_scan      — bulk BB register snapshot 0xC00-0xEFF, NONZERO/CHANGED
 *   c2h_log      — ring buffer of C2H firmware events (16 entries)
 *****************************************************************************/
#ifndef __PHYDM_CSI_H__
#define __PHYDM_CSI_H__

#include "phydm_precomp.h"

void phydm_csi_query(struct dm_struct *dm,
		     struct phydm_phyinfo_struct *phy_info,
		     u8 *phy_status_inf);

void phydm_bb_scan_trigger(struct dm_struct *dm);
void phydm_c2h_log_entry(u8 id, u8 seq, u8 plen, u8 *payload);

#ifdef CONFIG_PROC_DEBUG
void phydm_rx_phy_stats_proc_show(struct dm_struct *dm, void *m);
void phydm_bb_scan_proc_show(void *m);
void phydm_c2h_log_proc_show(void *m);
#endif

#endif /* __PHYDM_CSI_H__ */
