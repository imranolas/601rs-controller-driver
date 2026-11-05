# The controller's programming manual

The driver is built to the CNC-Control controller's programming manual, by B. Lewetz (SIG
Positec), in German. The manual is the vendor's and is not included here; it comes with GMC,
Gunnar's software for the 601. The protocol reference cites it by version and page.

| Document | Cited as |
|----------|----------|
| "CNC-Control — Bahnsteuerung für 4 Schrittmotorachsen", v1.00/00b, 20 Jul 2000. The newest version found, and the one the driver is built to. | [SPEC1 p.N] |
| The same manual at v0.99/69, 18 Apr 2000, a draft. Superseded by v1.00 where they differ. | [SPEC p.N] |
| A flowchart of the radius-dependent slowdown (`#C20`–`#C28`), 3 Mar 2003. | [RADIUS] |

The 601's controller runs firmware **2.00/55W** (19.10.10), newer than either version of the
manual. Where the captures or the sessions disagree with the manual, they win (protocol
reference §13.1).
