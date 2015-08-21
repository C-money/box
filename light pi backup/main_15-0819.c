/*
 * main.c
 *
 * Copyright (c) 2014 Jeremy Garff <jer @ jers.net>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     1.  Redistributions of source code must retain the above copyright notice, this list of
 *         conditions and the following disclaimer.
 *     2.  Redistributions in binary form must reproduce the above copyright notice, this list
 *         of conditions and the following disclaimer in the documentation and/or other materials
 *         provided with the distribution.
 *     3.  Neither the name of the owner nor the names of its contributors may be used to endorse
 *         or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>

#include "clk.h"
#include "gpio.h"
#include "dma.h"
#include "pwm.h"

#include "ws2811.h"

#include <errno.h>

#include "wiringSerial.h"

#define BCM2708_ST_BASE 0x20003000 /* BCM 2835 System Timer */

volatile unsigned *TIMER_registers;

unsigned int TIMER_GetSysTick()
{
    return TIMER_registers[1];
}

void TIMER_Init()
{
    /* open /dev/mem */
    int TIMER_memFd;
    if ((TIMER_memFd = open("/dev/mem", O_RDWR/*|O_SYNC*/) ) < 0)
    {
        printf("can't open /dev/mem - need root ?\n");
        exit(-1);
    }

    /* mmap BCM System Timer */
    void *TIMER_map = mmap(
        NULL,
        4096, /* BLOCK_SIZE */
        PROT_READ /*|PROT_WRITE*/,
        MAP_SHARED,
        TIMER_memFd,
        BCM2708_ST_BASE
    );

    close(TIMER_memFd);

    if (TIMER_map == MAP_FAILED)
    {
        printf("mmap error %d\n", (int)TIMER_map);
        exit(-1);
    }
    TIMER_registers = (volatile unsigned *)TIMER_map;
}


// Box LED Hub
// LED controller and slave to motion sense hub for box lighting effects

// Cole Hatton
// 15-0627
//  v0.3.2


uint16_t const cos_wave_larger[2048] ={ 2047 ,  2047 , 2047 , 2047 , 2047 , 2047 , 2047 , 2047 , 2047 , 2047 , 2047 , 2046 , 2046 , 2046 , 2046 , 2046 , 2046 , 2046 , 2045 , 2045 , 2045 , 2045 , 2045 , 2044 , 2044 , 2044 , 2044 , 2043 , 2043 , 2043 , 2043 , 2042 , 2042 , 2042 , 2041 , 2041 , 2041 , 2040 , 2040 , 2040 , 2039 , 2039 , 2039 , 2038 , 2038 , 2037 , 2037 , 2036 , 2036 , 2035 , 2035 , 2034 , 2034 , 2033 , 2033 , 2032 , 2032 , 2031 , 2031 , 2030 , 2030 , 2029 , 2029 , 2028 , 2027 , 2027 , 2026 , 2025 , 2025 , 2024 , 2023 , 2023 , 2022 , 2021 , 2021 , 2020 , 2019 , 2019 , 2018 , 2017 , 2016 , 2016 , 2015 , 2014 , 2013 , 2012 , 2012 , 2011 , 2010 , 2009 , 2008 , 2007 , 2007 , 2006 , 2005 , 2004 , 2003 , 2002 , 2001 , 2000 , 1999 , 1998 , 1997 , 1996 , 1995 , 1994 , 1993 , 1992 , 1991 , 1990 , 1989 , 1988 , 1987 , 1986 , 1985 , 1984 , 1983 , 1982 , 1981 , 1980 , 1978 , 1977 , 1976 , 1975 , 1974 , 1973 , 1971 , 1970 , 1969 , 1968 , 1967 , 1965 , 1964 , 1963 , 1962 , 1960 , 1959 , 1958 , 1957 , 1955 , 1954 , 1953 , 1951 , 1950 , 1949 , 1947 , 1946 , 1945 , 1943 , 1942 , 1941 , 1939 , 1938 , 1936 , 1935 , 1933 , 1932 , 1931 , 1929 , 1928 , 1926 , 1925 , 1923 , 1922 , 1920 , 1919 , 1917 , 1916 , 1914 , 1912 , 1911 , 1909 , 1908 , 1906 , 1905 , 1903 , 1901 , 1900 , 1898 , 1897 , 1895 , 1893 , 1892 , 1890 , 1888 , 1887 , 1885 , 1883 , 1881 , 1880 , 1878 , 1876 , 1875 , 1873 , 1871 , 1869 , 1867 , 1866 , 1864 , 1862 , 1860 , 1858 , 1857 , 1855 , 1853 , 1851 , 1849 , 1847 , 1846 , 1844 , 1842 , 1840 , 1838 , 1836 , 1834 , 1832 , 1830 , 1828 , 1826 , 1825 , 1823 , 1821 , 1819 , 1817 , 1815 , 1813 , 1811 , 1809 , 1807 , 1805 , 1803 , 1801 , 1799 , 1796 , 1794 , 1792 , 1790 , 1788 , 1786 , 1784 , 1782 , 1780 , 1778 , 1776 , 1773 , 1771 , 1769 , 1767 , 1765 , 1763 , 1760 , 1758 , 1756 , 1754 , 1752 , 1749 , 1747 , 1745 , 1743 , 1741 , 1738 , 1736 , 1734 , 1732 , 1729 , 1727 , 1725 , 1722 , 1720 , 1718 , 1715 , 1713 , 1711 , 1709 , 1706 , 1704 , 1701 , 1699 , 1697 , 1694 , 1692 , 1690 , 1687 , 1685 , 1682 , 1680 , 1678 , 1675 , 1673 , 1670 , 1668 , 1665 , 1663 , 1661 , 1658 , 1656 , 1653 , 1651 , 1648 , 1646 , 1643 , 1641 , 1638 , 1636 , 1633 , 1631 , 1628 , 1626 , 1623 , 1621 , 1618 , 1615 , 1613 , 1610 , 1608 , 1605 , 1603 , 1600 , 1597 , 1595 , 1592 , 1590 , 1587 , 1584 , 1582 , 1579 , 1576 , 1574 , 1571 , 1568 , 1566 , 1563 , 1560 , 1558 , 1555 , 1552 , 1550 , 1547 , 1544 , 1542 , 1539 , 1536 , 1533 , 1531 , 1528 , 1525 , 1523 , 1520 , 1517 , 1514 , 1512 , 1509 , 1506 , 1503 , 1500 , 1498 , 1495 , 1492 , 1489 , 1486 , 1484 , 1481 , 1478 , 1475 , 1472 , 1470 , 1467 , 1464 , 1461 , 1458 , 1455 , 1453 , 1450 , 1447 , 1444 , 1441 , 1438 , 1435 , 1433 , 1430 , 1427 , 1424 , 1421 , 1418 , 1415 , 1412 , 1409 , 1406 , 1404 , 1401 , 1398 , 1395 , 1392 , 1389 , 1386 , 1383 , 1380 , 1377 , 1374 , 1371 , 1368 , 1365 , 1362 , 1359 , 1356 , 1353 , 1351 , 1348 , 1345 , 1342 , 1339 , 1336 , 1333 , 1330 , 1327 , 1324 , 1321 , 1318 , 1315 , 1312 , 1309 , 1306 , 1303 , 1300 , 1296 , 1293 , 1290 , 1287 , 1284 , 1281 , 1278 , 1275 , 1272 , 1269 , 1266 , 1263 , 1260 , 1257 , 1254 , 1251 , 1248 , 1245 , 1242 , 1239 , 1235 , 1232 , 1229 , 1226 , 1223 , 1220 , 1217 , 1214 , 1211 , 1208 , 1205 , 1202 , 1198 , 1195 , 1192 , 1189 , 1186 , 1183 , 1180 , 1177 , 1174 , 1171 , 1167 , 1164 , 1161 , 1158 , 1155 , 1152 , 1149 , 1146 , 1143 , 1139 , 1136 , 1133 , 1130 , 1127 , 1124 , 1121 , 1118 , 1114 , 1111 , 1108 , 1105 , 1102 , 1099 , 1096 , 1093 , 1089 , 1086 , 1083 , 1080 , 1077 , 1074 , 1071 , 1067 , 1064 , 1061 , 1058 , 1055 , 1052 , 1049 , 1045 , 1042 , 1039 , 1036 , 1033 , 1030 , 1027 , 1024 , 1020 , 1017 , 1014 , 1011 , 1008 , 1005 , 1002 , 998 , 995 , 992 , 989 , 986 , 983 , 980 , 976 , 973 , 970 , 967 , 964 , 961 , 958 , 954 , 951 , 948 , 945 , 942 , 939 , 936 , 933 , 929 , 926 , 923 , 920 , 917 , 914 , 911 , 908 , 904 , 901 , 898 , 895 , 892 , 889 , 886 , 883 , 880 , 876 , 873 , 870 , 867 , 864 , 861 , 858 , 855 , 852 , 849 , 845 , 842 , 839 , 836 , 833 , 830 , 827 , 824 , 821 , 818 , 815 , 812 , 808 , 805 , 802 , 799 , 796 , 793 , 790 , 787 , 784 , 781 , 778 , 775 , 772 , 769 , 766 , 763 , 760 , 757 , 754 , 751 , 747 , 744 , 741 , 738 , 735 , 732 , 729 , 726 , 723 , 720 , 717 , 714 , 711 , 708 , 705 , 702 , 699 , 696 , 694 , 691 , 688 , 685 , 682 , 679 , 676 , 673 , 670 , 667 , 664 , 661 , 658 , 655 , 652 , 649 , 646 , 643 , 641 , 638 , 635 , 632 , 629 , 626 , 623 , 620 , 617 , 614 , 612 , 609 , 606 , 603 , 600 , 597 , 594 , 592 , 589 , 586 , 583 , 580 , 577 , 575 , 572 , 569 , 566 , 563 , 561 , 558 , 555 , 552 , 549 , 547 , 544 , 541 , 538 , 535 , 533 , 530 , 527 , 524 , 522 , 519 , 516 , 514 , 511 , 508 , 505 , 503 , 500 , 497 , 495 , 492 , 489 , 487 , 484 , 481 , 479 , 476 , 473 , 471 , 468 , 465 , 463 , 460 , 457 , 455 , 452 , 450 , 447 , 444 , 442 , 439 , 437 , 434 , 432 , 429 , 426 , 424 , 421 , 419 , 416 , 414 , 411 , 409 , 406 , 404 , 401 , 399 , 396 , 394 , 391 , 389 , 386 , 384 , 382 , 379 , 377 , 374 , 372 , 369 , 367 , 365 , 362 , 360 , 357 , 355 , 353 , 350 , 348 , 346 , 343 , 341 , 338 , 336 , 334 , 332 , 329 , 327 , 325 , 322 , 320 , 318 , 315 , 313 , 311 , 309 , 306 , 304 , 302 , 300 , 298 , 295 , 293 , 291 , 289 , 287 , 284 , 282 , 280 , 278 , 276 , 274 , 271 , 269 , 267 , 265 , 263 , 261 , 259 , 257 , 255 , 253 , 251 , 248 , 246 , 244 , 242 , 240 , 238 , 236 , 234 , 232 , 230 , 228 , 226 , 224 , 222 , 221 , 219 , 217 , 215 , 213 , 211 , 209 , 207 , 205 , 203 , 201 , 200 , 198 , 196 , 194 , 192 , 190 , 189 , 187 , 185 , 183 , 181 , 180 , 178 , 176 , 174 , 172 , 171 , 169 , 167 , 166 , 164 , 162 , 160 , 159 , 157 , 155 , 154 , 152 , 150 , 149 , 147 , 146 , 144 , 142 , 141 , 139 , 138 , 136 , 135 , 133 , 131 , 130 , 128 , 127 , 125 , 124 , 122 , 121 , 119 , 118 , 116 , 115 , 114 , 112 , 111 , 109 , 108 , 106 , 105 , 104 , 102 , 101 , 100 , 98 , 97 , 96 , 94 , 93 , 92 , 90 , 89 , 88 , 87 , 85 , 84 , 83 , 82 , 80 , 79 , 78 , 77 , 76 , 74 , 73 , 72 , 71 , 70 , 69 , 67 , 66 , 65 , 64 , 63 , 62 , 61 , 60 , 59 , 58 , 57 , 56 , 55 , 54 , 53 , 52 , 51 , 50 , 49 , 48 , 47 , 46 , 45 , 44 , 43 , 42 , 41 , 40 , 40 , 39 , 38 , 37 , 36 , 35 , 35 , 34 , 33 , 32 , 31 , 31 , 30 , 29 , 28 , 28 , 27 , 26 , 26 , 25 , 24 , 24 , 23 , 22 , 22 , 21 , 20 , 20 , 19 , 18 , 18 , 17 , 17 , 16 , 16 , 15 , 15 , 14 , 14 , 13 , 13 , 12 , 12 , 11 , 11 , 10 , 10 , 9 , 9 , 8 , 8 , 8 , 7 , 7 , 7 , 6 , 6 , 6 , 5 , 5 , 5 , 4 , 4 , 4 , 4 , 3 , 3 , 3 , 3 , 2 , 2 , 2 , 2 , 2 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 2 , 2 , 2 , 2 , 2 , 3 , 3 , 3 , 3 , 4 , 4 , 4 , 4 , 5 , 5 , 5 , 6 , 6 , 6 , 7 , 7 , 7 , 8 , 8 , 8 , 9 , 9 , 10 , 10 , 11 , 11 , 12 , 12 , 13 , 13 , 14 , 14 , 15 , 15 , 16 , 16 , 17 , 17 , 18 , 18 , 19 , 20 , 20 , 21 , 22 , 22 , 23 , 24 , 24 , 25 , 26 , 26 , 27 , 28 , 28 , 29 , 30 , 31 , 31 , 32 , 33 , 34 , 35 , 35 , 36 , 37 , 38 , 39 , 40 , 40 , 41 , 42 , 43 , 44 , 45 , 46 , 47 , 48 , 49 , 50 , 51 , 52 , 53 , 54 , 55 , 56 , 57 , 58 , 59 , 60 , 61 , 62 , 63 , 64 , 65 , 66 , 67 , 69 , 70 , 71 , 72 , 73 , 74 , 76 , 77 , 78 , 79 , 80 , 82 , 83 , 84 , 85 , 87 , 88 , 89 , 90 , 92 , 93 , 94 , 96 , 97 , 98 , 100 , 101 , 102 , 104 , 105 , 106 , 108 , 109 , 111 , 112 , 114 , 115 , 116 , 118 , 119 , 121 , 122 , 124 , 125 , 127 , 128 , 130 , 131 , 133 , 135 , 136 , 138 , 139 , 141 , 142 , 144 , 146 , 147 , 149 , 150 , 152 , 154 , 155 , 157 , 159 , 160 , 162 , 164 , 166 , 167 , 169 , 171 , 172 , 174 , 176 , 178 , 180 , 181 , 183 , 185 , 187 , 189 , 190 , 192 , 194 , 196 , 198 , 200 , 201 , 203 , 205 , 207 , 209 , 211 , 213 , 215 , 217 , 219 , 221 , 222 , 224 , 226 , 228 , 230 , 232 , 234 , 236 , 238 , 240 , 242 , 244 , 246 , 248 , 251 , 253 , 255 , 257 , 259 , 261 , 263 , 265 , 267 , 269 , 271 , 274 , 276 , 278 , 280 , 282 , 284 , 287 , 289 , 291 , 293 , 295 , 298 , 300 , 302 , 304 , 306 , 309 , 311 , 313 , 315 , 318 , 320 , 322 , 325 , 327 , 329 , 332 , 334 , 336 , 338 , 341 , 343 , 346 , 348 , 350 , 353 , 355 , 357 , 360 , 362 , 365 , 367 , 369 , 372 , 374 , 377 , 379 , 382 , 384 , 386 , 389 , 391 , 394 , 396 , 399 , 401 , 404 , 406 , 409 , 411 , 414 , 416 , 419 , 421 , 424 , 426 , 429 , 432 , 434 , 437 , 439 , 442 , 444 , 447 , 450 , 452 , 455 , 457 , 460 , 463 , 465 , 468 , 471 , 473 , 476 , 479 , 481 , 484 , 487 , 489 , 492 , 495 , 497 , 500 , 503 , 505 , 508 , 511 , 514 , 516 , 519 , 522 , 524 , 527 , 530 , 533 , 535 , 538 , 541 , 544 , 547 , 549 , 552 , 555 , 558 , 561 , 563 , 566 , 569 , 572 , 575 , 577 , 580 , 583 , 586 , 589 , 592 , 594 , 597 , 600 , 603 , 606 , 609 , 612 , 614 , 617 , 620 , 623 , 626 , 629 , 632 , 635 , 638 , 641 , 643 , 646 , 649 , 652 , 655 , 658 , 661 , 664 , 667 , 670 , 673 , 676 , 679 , 682 , 685 , 688 , 691 , 694 , 696 , 699 , 702 , 705 , 708 , 711 , 714 , 717 , 720 , 723 , 726 , 729 , 732 , 735 , 738 , 741 , 744 , 747 , 751 , 754 , 757 , 760 , 763 , 766 , 769 , 772 , 775 , 778 , 781 , 784 , 787 , 790 , 793 , 796 , 799 , 802 , 805 , 808 , 812 , 815 , 818 , 821 , 824 , 827 , 830 , 833 , 836 , 839 , 842 , 845 , 849 , 852 , 855 , 858 , 861 , 864 , 867 , 870 , 873 , 876 , 880 , 883 , 886 , 889 , 892 , 895 , 898 , 901 , 904 , 908 , 911 , 914 , 917 , 920 , 923 , 926 , 929 , 933 , 936 , 939 , 942 , 945 , 948 , 951 , 954 , 958 , 961 , 964 , 967 , 970 , 973 , 976 , 980 , 983 , 986 , 989 , 992 , 995 , 998 , 1002 , 1005 , 1008 , 1011 , 1014 , 1017 , 1020 , 1024 , 1027 , 1030 , 1033 , 1036 , 1039 , 1042 , 1045 , 1049 , 1052 , 1055 , 1058 , 1061 , 1064 , 1067 , 1071 , 1074 , 1077 , 1080 , 1083 , 1086 , 1089 , 1093 , 1096 , 1099 , 1102 , 1105 , 1108 , 1111 , 1114 , 1118 , 1121 , 1124 , 1127 , 1130 , 1133 , 1136 , 1139 , 1143 , 1146 , 1149 , 1152 , 1155 , 1158 , 1161 , 1164 , 1167 , 1171 , 1174 , 1177 , 1180 , 1183 , 1186 , 1189 , 1192 , 1195 , 1198 , 1202 , 1205 , 1208 , 1211 , 1214 , 1217 , 1220 , 1223 , 1226 , 1229 , 1232 , 1235 , 1239 , 1242 , 1245 , 1248 , 1251 , 1254 , 1257 , 1260 , 1263 , 1266 , 1269 , 1272 , 1275 , 1278 , 1281 , 1284 , 1287 , 1290 , 1293 , 1296 , 1300 , 1303 , 1306 , 1309 , 1312 , 1315 , 1318 , 1321 , 1324 , 1327 , 1330 , 1333 , 1336 , 1339 , 1342 , 1345 , 1348 , 1351 , 1353 , 1356 , 1359 , 1362 , 1365 , 1368 , 1371 , 1374 , 1377 , 1380 , 1383 , 1386 , 1389 , 1392 , 1395 , 1398 , 1401 , 1404 , 1406 , 1409 , 1412 , 1415 , 1418 , 1421 , 1424 , 1427 , 1430 , 1433 , 1435 , 1438 , 1441 , 1444 , 1447 , 1450 , 1453 , 1455 , 1458 , 1461 , 1464 , 1467 , 1470 , 1472 , 1475 , 1478 , 1481 , 1484 , 1486 , 1489 , 1492 , 1495 , 1498 , 1500 , 1503 , 1506 , 1509 , 1512 , 1514 , 1517 , 1520 , 1523 , 1525 , 1528 , 1531 , 1533 , 1536 , 1539 , 1542 , 1544 , 1547 , 1550 , 1552 , 1555 , 1558 , 1560 , 1563 , 1566 , 1568 , 1571 , 1574 , 1576 , 1579 , 1582 , 1584 , 1587 , 1590 , 1592 , 1595 , 1597 , 1600 , 1603 , 1605 , 1608 , 1610 , 1613 , 1615 , 1618 , 1621 , 1623 , 1626 , 1628 , 1631 , 1633 , 1636 , 1638 , 1641 , 1643 , 1646 , 1648 , 1651 , 1653 , 1656 , 1658 , 1661 , 1663 , 1665 , 1668 , 1670 , 1673 , 1675 , 1678 , 1680 , 1682 , 1685 , 1687 , 1690 , 1692 , 1694 , 1697 , 1699 , 1701 , 1704 , 1706 , 1709 , 1711 , 1713 , 1715 , 1718 , 1720 , 1722 , 1725 , 1727 , 1729 , 1732 , 1734 , 1736 , 1738 , 1741 , 1743 , 1745 , 1747 , 1749 , 1752 , 1754 , 1756 , 1758 , 1760 , 1763 , 1765 , 1767 , 1769 , 1771 , 1773 , 1776 , 1778 , 1780 , 1782 , 1784 , 1786 , 1788 , 1790 , 1792 , 1794 , 1796 , 1799 , 1801 , 1803 , 1805 , 1807 , 1809 , 1811 , 1813 , 1815 , 1817 , 1819 , 1821 , 1823 , 1825 , 1826 , 1828 , 1830 , 1832 , 1834 , 1836 , 1838 , 1840 , 1842 , 1844 , 1846 , 1847 , 1849 , 1851 , 1853 , 1855 , 1857 , 1858 , 1860 , 1862 , 1864 , 1866 , 1867 , 1869 , 1871 , 1873 , 1875 , 1876 , 1878 , 1880 , 1881 , 1883 , 1885 , 1887 , 1888 , 1890 , 1892 , 1893 , 1895 , 1897 , 1898 , 1900 , 1901 , 1903 , 1905 , 1906 , 1908 , 1909 , 1911 , 1912 , 1914 , 1916 , 1917 , 1919 , 1920 , 1922 , 1923 , 1925 , 1926 , 1928 , 1929 , 1931 , 1932 , 1933 , 1935 , 1936 , 1938 , 1939 , 1941 , 1942 , 1943 , 1945 , 1946 , 1947 , 1949 , 1950 , 1951 , 1953 , 1954 , 1955 , 1957 , 1958 , 1959 , 1960 , 1962 , 1963 , 1964 , 1965 , 1967 , 1968 , 1969 , 1970 , 1971 , 1973 , 1974 , 1975 , 1976 , 1977 , 1978 , 1980 , 1981 , 1982 , 1983 , 1984 , 1985 , 1986 , 1987 , 1988 , 1989 , 1990 , 1991 , 1992 , 1993 , 1994 , 1995 , 1996 , 1997 , 1998 , 1999 , 2000 , 2001 , 2002 , 2003 , 2004 , 2005 , 2006 , 2007 , 2007 , 2008 , 2009 , 2010 , 2011 , 2012 , 2012 , 2013 , 2014 , 2015 , 2016 , 2016 , 2017 , 2018 , 2019 , 2019 , 2020 , 2021 , 2021 , 2022 , 2023 , 2023 , 2024 , 2025 , 2025 , 2026 , 2027 , 2027 , 2028 , 2029 , 2029 , 2030 , 2030 , 2031 , 2031 , 2032 , 2032 , 2033 , 2033 , 2034 , 2034 , 2035 , 2035 , 2036 , 2036 , 2037 , 2037 , 2038 , 2038 , 2039 , 2039 , 2039 , 2040 , 2040 , 2040 , 2041 , 2041 , 2041 , 2042 , 2042 , 2042 , 2043 , 2043 , 2043 , 2043 , 2044 , 2044 , 2044 , 2044 , 2045 , 2045 , 2045 , 2045 , 2045 , 2046 , 2046 , 2046 , 2046 , 2046 , 2046 , 2046 , 2047 , 2047 , 2047 , 2047 , 2047 , 2047 , 2047 , 2047 , 2047 , 2047 };

//Gamma Correction Curve
uint8_t const exp_gamma[256] =
{0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,3,3,3,3,3,
4,4,4,4,4,5,5,5,5,5,6,6,6,7,7,7,7,8,8,8,9,9,9,10,10,10,11,11,12,12,12,13,13,14,14,14,15,15,
16,16,17,17,18,18,19,19,20,20,21,21,22,23,23,24,24,25,26,26,27,28,28,29,30,30,31,32,32,33,
34,35,35,36,37,38,39,39,40,41,42,43,44,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,
61,62,63,64,65,66,67,68,70,71,72,73,74,75,77,78,79,80,82,83,84,85,87,89,91,92,93,95,96,98,
99,100,101,102,105,106,108,109,111,112,114,115,117,118,120,121,123,125,126,128,130,131,133,
135,136,138,140,142,143,145,147,149,151,152,154,156,158,160,162,164,165,167,169,171,173,175,
177,179,181,183,185,187,190,192,194,196,198,200,202,204,207,209,211,213,216,218,220,222,225,
227,229,232,234,236,239,241,244,246,249,251,253,254,255
};


inline uint16_t fastCosineCalc( uint16_t preWrapVal)
{
  return cos_wave_larger[preWrapVal & 2047];//(pgm_read_byte_near(cos_wave_16+wrapVal)); 
}



#define TRUE 1
#define FALSE 0

#define LONGEST_STRIP 110// all of the following params need to be adjusted for screen size
#define N_LED_OUTPUTS 3  // LED_LAYOUT assumed 0 if ROWS_LEDs > 8
#define N_LEDS    700 //(LONGEST_STRIP * N_LED_OUTPUTS)

#define N_STRIPS 23


#define ARRAY_SIZE(stuff)                        (sizeof(stuff) / sizeof(stuff[0]))

#define TARGET_FREQ                              WS2811_TARGET_FREQ
#define GPIO_PIN                                 18
#define DMA                                      5

#define WIDTH                                    N_LEDS
#define HEIGHT                                   1
#define LED_COUNT                                (WIDTH * HEIGHT)


ws2811_t ledstring =
{
    .freq = TARGET_FREQ,
    .dmanum = DMA,
    .channel =
    {
        [0] =
        {
            .gpionum = GPIO_PIN,
            .count = LED_COUNT,
            .invert = 0,
            .brightness = 255,
        },
        [1] =
        {
            .gpionum = 0,
            .count = 0,
            .invert = 0,
            .brightness = 0,
        },
    },
};

ws2811_led_t matrix[WIDTH];


void matrix_render(void)
{
    int x;

    for (x = 0; x < WIDTH; x++)
    {
       ledstring.channel[0].leds[x] = matrix[x];
    }
}


void SetBrightness(int motion_data)
{
  int x;
    for (x = 0; x < WIDTH; x++) {
      (matrix[x]) = motion_data;
    }
}
  

int dotspos[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
ws2811_led_t dotcolors[] =
{
    0x200000,  // red
    0x201000,  // orange
    0x202000,  // yellow
    0x002000,  // green
    0x002020,  // lightblue
    0x000020,  // blue
    0x100010,  // purple
    0x200010,  // pink
};


static void ctrl_c_handler(int signum)
{
    ws2811_fini(&ledstring);
}

static void setup_handlers(void)
{
    struct sigaction sa =
    {
        .sa_handler = ctrl_c_handler,
    };

    sigaction(SIGKILL, &sa, NULL);
}



//real vals, index off by 1 - int strip_lengths[N_STRIPS] = {21, 27, 27, 34, 29, 38, 29, 23, 29, 28, 24, 30, 28};
//int strip_lengths[N_STRIPS] = {21, 27, 27, 34, 29, 38, 29, 23, 48, 29, 24, 30, 55};
// new remapped values after for first box test
int strip_lengths[N_STRIPS] ={24, 28, 30, 29, 29, 34, 30, 38, 23, 21, 31, 33, 35, 29, 30, 33, 30, 26, 24, 36, 38, 31, 27}; //, 27, 27, 35, 38, 29, 43, 30, 28, 28, 24};
int strip_x[N_STRIPS] = {60, 72, 92, 80, 80, 85, 79, 66, 39, 28, 26, 12, 8, 25, 10, 12, 26, 11, 7, 25, 13, 28, 44};
int strip_y[N_STRIPS] = {24, 25, 8, 32, 49, 66, 74, 76, 85, 78, 90, 88, 77, 65, 59, 46, 41, 35, 25, 23, 12, 5, 23};

#define STRIP_RED_LEVEL_MAX  1024
#define STRIP_RED_LEVEL_DEC  2
int strip_red_levels[N_STRIPS];// = {0, 0, 0, 0, 0, 0};
int strip_red_setpoints[N_STRIPS];// = {0, 0, 0, 0, 0, 0};

#define N_MOT_SENSORS 23


unsigned long frameCount = 500;  // arbitrary seed to calculate the three time displacement variables t,t2,t3

int incomingByte;
//Byte val 2PI Cosine Wave, offset by 1 PI 
//supports fast trig calcs and smooth LED fading/pulsing

static void BluePlasmaStep(void);
static void FireStep(void);
static void SolidColorsStep(void);
static void LightningStep(void);
static void SolidDarksStep(void);
static void RGBFlashStep(void);
static void SolidAllStep(void);
static void StaticStep(void);
static void RainbowStep(void);
static void StripLengthTestStep(void);
static void XSweep(void);
static void YSweep(void);
static void ZSweep(void);

static void InitFire(void);
static void InitSolidColors(void);
static void InitLightning(void);
static void InitSolidDarks(void);
static void CheckSceneChangeKeys(int key_pressed);

static uint8_t motion_data[31];
int fd_key;
struct input_event ev;

static int scene = 0;
static int scene_override = 0;
#define N_SCENES     9
#define BLUE_PLASMA  0
#define FIRE         1
#define SOLID_COLORS 2
#define LIGHTNING    3
#define SOLID_DARKS  4
#define RGB_FLASH    5
#define SOLID_ALL    6
#define STATICS      7
#define RAINBOW      8

int main(int argc, char *argv[])
{
  int ret = 0;
  int i;
  static uint8_t motion_data_read = 0;
  static uint8_t motion_data_index = 0;
  int loop_count = 0;
  long int this_time, last_time;
  long int time_difference;
  //struct timespec gettime_now;
  setup_handlers();

  fd_key = open("/dev/input/event0", O_RDONLY | O_NONBLOCK);

  if (ws2811_init(&ledstring))
    {
      return -1;
    }

  int fd ;

  printf("Opening Serial\n");

  TIMER_Init();

  // if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
  // if ((fd = serialOpen ("/dev/serial/by-id/usb-Teensyduino_USB_Serial_847320-if00", 115200)) < 0)
  if ((fd = serialOpen("/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0", 115200)) < 0)
    {
 
      fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
      return 1 ;
    }


  printf("start\n");

  InitSolidColors();
  InitFire();
  InitLightning();
  InitSolidDarks();

  //  clock_gettime(CLOCK_REALTIME, &gettime_now);
  last_time = TIMER_GetSysTick(); //gettime_now.tv_nsec;


  while (1)
    {
      loop_count++;
      if (loop_count == 100) {
	//  clock_gettime(CLOCK_REALTIME, &gettime_now);
	this_time = TIMER_GetSysTick(); //gettime_now.tv_nsec;
	time_difference = this_time - last_time;
	printf("dt [us]: %7lu\n", time_difference);
	last_time = this_time;
	loop_count = 0;
	for (i = 0; i < 10; i++) {
	  // printf("%i\n", motion_data[i]);
	}
      }

      while (serialDataAvail (fd)) {
	motion_data_read = serialGetchar(fd);
	if (motion_data_read == 254) {
	  motion_data_index = 0;
	} else {
	  if (motion_data_read == 255) motion_data_read = 0;
	  motion_data[motion_data_index] = motion_data_read;
	  motion_data_index++;
	}
	//printf (" -> %3d\n", (uint16_t)motion_data_read);
	//SetBrightness(motion_data[1]);
	fflush (stdout) ;
      }

      if (scene_override == 0) {
	scene = motion_data[0];
      }


      // A FAT RED LINE OF TEXT ============================================
      
      if (FALSE) {   // kill compiler warning on unused fn
	StripLengthTestStep();
	XSweep();
	YSweep();
	ZSweep();
	BluePlasmaStep();
	FireStep();
	SolidColorsStep();
	LightningStep();
	SolidDarksStep();
	RGBFlashStep();
	SolidAllStep();
	StaticStep();
	RainbowStep();
      }
      //StripLengthTestStep();
      switch (scene) {
      case BLUE_PLASMA:
      	BluePlasmaStep();
      	break;
      case FIRE:
      	FireStep();
      	break;
      case SOLID_COLORS:
      	SolidColorsStep();
      	break;
      case LIGHTNING:
      	LightningStep();
      	break;
      case SOLID_DARKS:
      	SolidDarksStep();
      	break;
      case RGB_FLASH:
      	RGBFlashStep();
      	break;
      case SOLID_ALL:
	SolidAllStep();
	break;
      case STATICS:
	StaticStep();
	break;
      case RAINBOW:
	RainbowStep();
	break;
      }
      //ZSweep();
     
      matrix_render();
      if (ws2811_render(&ledstring))
        {
	  ret = -1;
	  break;
        }

      // 15 frames /sec
      usleep(1000);//(1000000 /1000);

    }

  ws2811_fini(&ledstring);

  return ret;
}



static void CheckSceneChangeKeys(int key_pressed)
{
  switch (key_pressed) {
  case 13:   // =
    scene++;
    if (scene == N_SCENES) {
      scene = 0;
    }
    printf("Scene: %i\n\n", scene);
    break;
  case 12:   // -
    scene--;
    if (scene == -1) {
      scene = N_SCENES - 1;
    }
    printf("Scene: %i\n\n", scene);
    break;
  case 11:  // 0
    scene_override ^= 1;
    printf("Scene override: %i\n\n", scene_override);
    break;
  }
}

static void BluePlasmaStep(void)
{
  int strip_index, led_index;
  uint16_t i, x, z, r, g, b, pos1, pos2, pos3, tpos1, tpos2, tpos3;
  static long t1 = 0;
  static long t2 = 0;
  static long t3 = 0;
  static long t1_speed = 57;
  static long t2_speed = -91;
  static long t3_speed = 61;
  static int t_scale = 50;
  static int space_scale = 50;

  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n\n", ev.code, ev.value);
    if (ev.value == 1 || ev.value == 2) { // key press

      CheckSceneChangeKeys(ev.code);     // check for scene change hotkey

      if (ev.code == 105) { // left arrow
	if (t_scale > 1) {
	  t_scale--;
	}
	printf("time scale: %i\n\n", t_scale); 
      } else if (ev.code == 106) { // right arrow
	if (t_scale < 500) {
	  t_scale++;
	}
	printf("time scale: %i\n\n", t_scale);
      }
      else if (ev.code == 103) { // up arrow
	if (space_scale > 1) {
	  space_scale--;
	}
	printf("space scale: %i\n\n", space_scale);
      } else if (ev.code == 108) { // down arrow
	if (space_scale < 500) {
	  space_scale++;
	}
	printf("space scale: %i\n\n", space_scale);
      }
      else if (ev.code == 16) { // q
	if (t1_speed < 500) {
	  t1_speed++;
	}
	printf("t1 speed: %li\n\n", t1_speed);
      } else if (ev.code == 108) { // a
	if (t1_speed > -500) {
	  t1_speed--;
	}
	printf("t1 speed: %li\n\n", t1_speed);
      }
      else if (ev.code == 16) { // w
	if (t2_speed < 500) {
	  t2_speed++;
	}
	printf("t2 speed: %li\n\n", t2_speed);
      } else if (ev.code == 108) { // s
	if (t2_speed > -500) {
	  t2_speed--;
	}
	printf("t2 speed: %li\n\n", t2_speed);
      } 
      else if (ev.code == 16) { // e
	if (t3_speed < 500) {
	  t3_speed++;
	}
	printf("t3 speed: %li\n\n", t3_speed);
      } else if (ev.code == 108) { // d
	if (t3_speed > -500) {
	  t3_speed--;
	}
	printf("t3 speed: %li\n\n", t3_speed);
      }
      else if (ev.code == 25) { // p
	printf("time scale: %i\n", t_scale);
	printf("space scale: %i\n", space_scale);
	printf("t1 speed: %li\n", t1_speed);
	printf("t2 speed: %li\n", t2_speed);
	printf("t3 speed: %li\n", t3_speed);
	
      }
    }
  }


  t1 += (t1_speed * t_scale);
  t2 += (t2_speed * t_scale);
  t3 += (t3_speed * t_scale);
  tpos1 = fastCosineCalc(t1 >> 10);
  tpos2 = fastCosineCalc(t2 >> 10);
  tpos3 = fastCosineCalc(t3 >> 10);
  // frameCount+=10;
  //t = fastCosineCalc((43 * frameCount)/50);  //time displacement - fiddle with these til it looks good...
  //t2 = -fastCosineCalc((35 * frameCount)/50); 
  //t3 = fastCosineCalc((37 * frameCount)/50);
  
    
  for (i = 0; i < N_MOT_SENSORS; i++) {
    if (strip_red_setpoints[i] < motion_data[i + 1] * 8) {
      strip_red_setpoints[i] = motion_data[i + 1] * 8;
    }
  }
  //}
  
  for (i = 0; i < N_STRIPS; i++) {
    if (strip_red_setpoints[i] > STRIP_RED_LEVEL_DEC) {
      strip_red_setpoints[i] -= STRIP_RED_LEVEL_DEC;
    } else {
      strip_red_setpoints[i] = 0;
    }
    strip_red_levels[i] = (strip_red_setpoints[i] + strip_red_levels[i] * 15) >> 4;    // recursive set point following
  }
  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }
    z = strip_lengths[strip_index] - led_index;
    pos1 = ((-strip_x[strip_index] + strip_y[strip_index] + z) * 10 * space_scale) >> 6;
    pos2 = ((strip_x[strip_index] - strip_y[strip_index] + z) * 6 * space_scale) >> 6;
    pos3 = ((strip_x[strip_index] + strip_y[strip_index] - z) * 8 * space_scale) >> 6;
    //Calculate 3 seperate plasma waves, one for each color channel
    //r = fastCosineCalc(((x*20) + (t3 >> 1) + fastCosineCalc(t2 + (x*20))));
    //g = fastCosineCalc((t + (x*20) + fastCosineCalc((-(t3 >> 2) + (x*20)))));
    //b = fastCosineCalc((t2 + (x*20) + fastCosineCalc((t + (x*20) + 0*(g >> 2)))));
    r = fastCosineCalc((pos1 + (tpos3 >> 1) + fastCosineCalc(tpos2 + pos2)));
    g = fastCosineCalc((tpos1 + pos2 + fastCosineCalc((-(tpos3 >> 2) + pos3))));
    b = fastCosineCalc((tpos2 + pos3 + fastCosineCalc((tpos1 + pos1))));

    r = ((uint32_t)r * strip_red_levels[strip_index]) >> 13 ;
    g = g >> 3;
    b = b >> 3;    // bit shift of at least 3 needed for 8 bit color
    /*if (swap) {
      r = 255;
      g = 255;
      b = 255;
      } else {
      r = 0;
      b = 0;
      g = 0;
      }*/
    matrix[x] = (r << 16) + (g << 8) + b;
  }
}

static void RainbowStep(void)
{
  int strip_index, led_index;
  uint16_t i, x, z, pos1, pos2, pos3, tpos1, tpos2, tpos3;
  long r, g, b;
  static long t1 = 0;
  static long t2 = 0;
  static long t3 = 0;
  static long t1_speed = 57;
  static long t2_speed = -91;
  static long t3_speed = 61;
  static int t_scale = 12;
  static int space_scale = 22;

  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n\n", ev.code, ev.value);
    if (ev.value == 1 || ev.value == 2) { // key press

      CheckSceneChangeKeys(ev.code);     // check for scene change hotkey

      if (ev.code == 105) { // left arrow
	if (t_scale > 1) {
	  t_scale--;
	}
	printf("time scale: %i\n\n", t_scale); 
      } else if (ev.code == 106) { // right arrow
	if (t_scale < 500) {
	  t_scale++;
	}
	printf("time scale: %i\n\n", t_scale);
      }
      else if (ev.code == 103) { // up arrow
	if (space_scale > 1) {
	  space_scale--;
	}
	printf("space scale: %i\n\n", space_scale);
      } else if (ev.code == 108) { // down arrow
	if (space_scale < 500) {
	  space_scale++;
	}
	printf("space scale: %i\n\n", space_scale);
      }
      else if (ev.code == 16) { // q
	if (t1_speed < 500) {
	  t1_speed++;
	}
	printf("t1 speed: %li\n\n", t1_speed);
      } else if (ev.code == 108) { // a
	if (t1_speed > -500) {
	  t1_speed--;
	}
	printf("t1 speed: %li\n\n", t1_speed);
      }
      else if (ev.code == 16) { // w
	if (t2_speed < 500) {
	  t2_speed++;
	}
	printf("t2 speed: %li\n\n", t2_speed);
      } else if (ev.code == 108) { // s
	if (t2_speed > -500) {
	  t2_speed--;
	}
	printf("t2 speed: %li\n\n", t2_speed);
      } 
      else if (ev.code == 16) { // e
	if (t3_speed < 500) {
	  t3_speed++;
	}
	printf("t3 speed: %li\n\n", t3_speed);
      } else if (ev.code == 108) { // d
	if (t3_speed > -500) {
	  t3_speed--;
	}
	printf("t3 speed: %li\n\n", t3_speed);
      }
      else if (ev.code == 25) { // p
	printf("time scale: %i\n", t_scale);
	printf("space scale: %i\n", space_scale);
	printf("t1 speed: %li\n", t1_speed);
	printf("t2 speed: %li\n", t2_speed);
	printf("t3 speed: %li\n", t3_speed);
	
      }
    }
  }

  if (scene != RAINBOW) {
    t_scale = 12;
    space_scale = 22;
  }


  t1 += (t1_speed * t_scale);
  t2 += (t2_speed * t_scale);
  t3 += (t3_speed * t_scale);
  tpos1 = fastCosineCalc(t1 >> 10);
  tpos2 = fastCosineCalc(t2 >> 10);
  tpos3 = fastCosineCalc(t3 >> 10);
  // frameCount+=10;
  //t = fastCosineCalc((43 * frameCount)/50);  //time displacement - fiddle with these til it looks good...
  //t2 = -fastCosineCalc((35 * frameCount)/50); 
  //t3 = fastCosineCalc((37 * frameCount)/50);
  
    
  for (i = 0; i < N_MOT_SENSORS; i++) {
    
  }
  //}
  
  for (i = 0; i < N_STRIPS; i++) {
    
  }
  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }
    z = strip_lengths[strip_index] - led_index;
    pos1 = ((-strip_x[strip_index] + strip_y[strip_index] + z) * 10 * space_scale) >> 6;
    pos2 = ((strip_x[strip_index] - strip_y[strip_index] + z) * 6 * space_scale) >> 6;
    pos3 = ((strip_x[strip_index] + strip_y[strip_index] - z) * 8 * space_scale) >> 6;
    //Calculate 3 seperate plasma waves, one for each color channel
    //r = fastCosineCalc(((x*20) + (t3 >> 1) + fastCosineCalc(t2 + (x*20))));
    //g = fastCosineCalc((t + (x*20) + fastCosineCalc((-(t3 >> 2) + (x*20)))));
    //b = fastCosineCalc((t2 + (x*20) + fastCosineCalc((t + (x*20) + 0*(g >> 2)))));
    r = fastCosineCalc((pos1 + (tpos3 >> 1) + fastCosineCalc(tpos2 + pos2)));
    g = fastCosineCalc((tpos1 + pos2 + fastCosineCalc((-(tpos3 >> 2) + pos3))));
    b = fastCosineCalc((tpos2 + pos3 + fastCosineCalc((tpos1 + pos1))));

    r = (r * r) >> 14;
    g = (g * g) >> 14;
    b = (b * b) >> 14;    // bit shift of at least 3 needed for 8 bit color
    /*if (swap) {
      r = 255;
      g = 255;
      b = 255;
      } else {
      r = 0;
      b = 0;
      g = 0;
      }*/
    matrix[x] = (r << 16) + (g << 8) + b;
  }
}

#define BRIGHT_SLOPE_BASE    1024
#define BRIGHT_SLOPE_MAX     2000
#define BRIGHT_SLOPE_MIN     300
#define TAP_TO_SLOPE_SCALE   14   
#define BRIGHT_SLOPE_STEP    5

static int strip_bright_slopes[N_STRIPS];
static int strip_bright_slope_setpoints[N_STRIPS];

static void InitFire(void)
{
  int i;
  for (i = 0; i < N_STRIPS; i++) {
    strip_bright_slopes[i] = BRIGHT_SLOPE_MAX;
    strip_bright_slope_setpoints[i] = BRIGHT_SLOPE_MAX;
  }
}

static void FireStep(void)
{
  int strip_index, led_index;
  uint16_t i, x, z, pos1, pos2, pos3, tpos1, tpos2, tpos3, next_slope;
  long r, g, b;
  long bright_scale, color_shift_strength, color_base_strength;
  static long t1 = 0;
  static long t2 = 0;
  static long t3 = 0;
  static long t1_speed = 57;
  static long t2_speed = 101;
  static long t3_speed = 61;
  static int t_scale = 120;
  static int space_scale = 300;

  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n\n", ev.code, ev.value);
    if (ev.value == 1 || ev.value == 2) { // key press

      CheckSceneChangeKeys(ev.code);     // check for scene change hotkey

      if (ev.code == 105) { // left arrow
	if (t_scale > 1) {
	  t_scale--;
	}
	printf("time scale: %i\n\n", t_scale); 
      } else if (ev.code == 106) { // right arrow
	if (t_scale < 500) {
	  t_scale++;
	}
	printf("time scale: %i\n\n", t_scale);
      }
      else if (ev.code == 103) { // up arrow
	if (space_scale > 20) {
	  space_scale-=20;
	}
	printf("space scale: %i\n\n", space_scale);
      } else if (ev.code == 108) { // down arrow
	if (space_scale < 5000) {
	  space_scale+=20;
	}
	printf("space scale: %i\n\n", space_scale);
      }
      else if (ev.code == 16) { // q
	if (t1_speed < 500) {
	  t1_speed++;
	}
	printf("t1 speed: %li\n\n", t1_speed);
      } else if (ev.code == 30) { // a
	if (t1_speed > -500) {
	  t1_speed--;
	}
	printf("t1 speed: %li\n\n", t1_speed);
      }
      else if (ev.code == 17) { // w
	if (t2_speed < 500) {
	  t2_speed++;
	}
	printf("t2 speed: %li\n\n", t2_speed);
      } else if (ev.code == 31) { // s
	if (t2_speed > -500) {
	  t2_speed--;
	}
	printf("t2 speed: %li\n\n", t2_speed);
      } 
      else if (ev.code == 18) { // e
	if (t3_speed < 500) {
	  t3_speed++;
	}
	printf("t3 speed: %li\n\n", t3_speed);
      } else if (ev.code == 32) { // d
	if (t3_speed > -500) {
	  t3_speed--;
	}
	printf("t3 speed: %li\n\n", t3_speed);
      }
      else if (ev.code == 25) { // p
	printf("time scale: %i\n", t_scale);
	printf("space scale: %i\n", space_scale);
	printf("t1 speed: %li\n", t1_speed);
	printf("t2 speed: %li\n", t2_speed);
	printf("t3 speed: %li\n", t3_speed);
	
      }
    }
  }


  t1 += (t1_speed * t_scale);
  t2 += (t2_speed * t_scale);
  t3 += (t3_speed * t_scale);
  //tpos1 = fastCosineCalc(t1 >> 10);
  //tpos2 = fastCosineCalc(t2 >> 10);
  //tpos3 = fastCosineCalc(t3 >> 10);
  tpos1 = t1 >> 10;  // rise continuously instead of circling
  tpos2 = t2 >> 10;
  tpos3 = t3 >> 10;
  // frameCount+=10;
  //t = fastCosineCalc((43 * frameCount)/50);  //time displacement - fiddle with these til it looks good...
  //t2 = -fastCosineCalc((35 * frameCount)/50); 
  //t3 = fastCosineCalc((37 * frameCount)/50);
  
    
  for (i = 0; i < N_MOT_SENSORS; i++) {
    if (strip_red_setpoints[i] < motion_data[i + 1] * 8) {
      strip_red_setpoints[i] = motion_data[i + 1] * 8;
    }
    next_slope = BRIGHT_SLOPE_MAX - (motion_data[i + 1] * TAP_TO_SLOPE_SCALE);
    if (next_slope < BRIGHT_SLOPE_MIN) {
      next_slope = BRIGHT_SLOPE_MIN;
    }
    if (next_slope < strip_bright_slope_setpoints[i]) {
      strip_bright_slope_setpoints[i] = next_slope;
    }
  }
  //}
  
  for (i = 0; i < N_STRIPS; i++) {
    if (strip_red_setpoints[i] > STRIP_RED_LEVEL_DEC) {
      strip_red_setpoints[i] -= STRIP_RED_LEVEL_DEC;
    } else {
      strip_red_setpoints[i] = 0;
    }
    strip_red_levels[i] = (strip_red_setpoints[i] + strip_red_levels[i] * 15) >> 4;    // recursive set point following
    if (strip_bright_slope_setpoints[i] < BRIGHT_SLOPE_MAX - BRIGHT_SLOPE_STEP) {
      (strip_bright_slope_setpoints[i]) += BRIGHT_SLOPE_STEP;
    } else {
      (strip_bright_slope_setpoints[i]) = BRIGHT_SLOPE_MAX; 
    }
    strip_bright_slopes[i] = (strip_bright_slope_setpoints[i] + strip_bright_slopes[i] * 15) >> 4;
  }

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }
    z = strip_lengths[strip_index] - led_index;
    pos1 = ((-strip_x[strip_index] + strip_y[strip_index] + z) * 10 * space_scale) >> 6;
    pos2 = ((strip_x[strip_index] - strip_y[strip_index] + z) * 6 * space_scale) >> 6;
    pos3 = ((-strip_x[strip_index] - strip_y[strip_index] + z) * 8 * space_scale) >> 6;
    //Calculate 3 seperate plasma waves, one for each color channel
    //r = fastCosineCalc(((x*20) + (t3 >> 1) + fastCosineCalc(t2 + (x*20))));
    //g = fastCosineCalc((t + (x*20) + fastCosineCalc((-(t3 >> 2) + (x*20)))));
    //b = fastCosineCalc((t2 + (x*20) + fastCosineCalc((t + (x*20) + 0*(g >> 2)))));
    r = fastCosineCalc((pos1 + (tpos3 >> 1) + fastCosineCalc(tpos2 + pos2)));
    g = fastCosineCalc((tpos1 + pos2 + fastCosineCalc(((tpos3 >> 2) + pos3))));
    b = fastCosineCalc((tpos2 + pos3 + fastCosineCalc((tpos1 + pos1))));

    bright_scale = (BRIGHT_SLOPE_BASE * strip_lengths[strip_index] - strip_bright_slopes[strip_index] * led_index);
    if (bright_scale < 0) {
      bright_scale = 0;
    }
    color_shift_strength = (256 * (BRIGHT_SLOPE_MAX - strip_bright_slopes[strip_index])) / (BRIGHT_SLOPE_MAX - BRIGHT_SLOPE_MIN);
    color_base_strength = (256 * (strip_bright_slopes[strip_index] - BRIGHT_SLOPE_MIN)) / (BRIGHT_SLOPE_MAX - BRIGHT_SLOPE_MIN);
    g = ((((g * r * color_base_strength) >> 23) + ((g * color_shift_strength) >> 12)) * bright_scale) / (BRIGHT_SLOPE_BASE * strip_lengths[strip_index]);
    b = (((color_shift_strength * b * r) >> 25) * bright_scale) / (BRIGHT_SLOPE_BASE * strip_lengths[strip_index]);
    r = ((r / 10 + 50) * bright_scale) / (BRIGHT_SLOPE_BASE * strip_lengths[strip_index]);
   

    

    //r = ((uint32_t)r * strip_red_levels[strip_index]) >> 13 ;
    //g = g >> 4;
    //b = b >> 4;    // bit shift of at least 3 needed for 8 bit color
    /*if (swap) {
      r = 255;
      g = 255;
      b = 255;
      } else {
      r = 0;
      b = 0;
      g = 0;
      }*/
    matrix[x] = (r << 16) + (g << 8) + b;
  }
}

#define LIGHTNING_PROB_MIN  6
#define LIGHTNING_PROB_MAX  250
#define LIGHTNING_PROB_STEP 2
#define TAP_TO_PROB_SCALE   2

static int strip_lightning_states[N_STRIPS];
static int strip_lightning_probs[N_STRIPS];

static void InitLightning(void)
{
  int i;
  for (i = 0; i < N_STRIPS; i++) {
    strip_lightning_probs[i] = LIGHTNING_PROB_MIN;
  }
}

static void LightningStep(void)
{
  int strip_index, led_index;
  uint16_t  i, x, r, g, b, next_prob;
  r = g = b = 0;

  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n\n", ev.code, ev.value);
    if (ev.value == 1 || ev.value == 2) { // key press

      CheckSceneChangeKeys(ev.code);     // check for scene change hotkey
    }
  } 

  for (i = 0; i < N_MOT_SENSORS; i++) {
    next_prob = LIGHTNING_PROB_MIN + (motion_data[i + 1] * TAP_TO_PROB_SCALE);
    if (next_prob > LIGHTNING_PROB_MAX) {
      next_prob = LIGHTNING_PROB_MAX;
    }
    if (next_prob > strip_lightning_probs[i]) {
      strip_lightning_probs[i] = next_prob;
    }
  }
  //}
  
  for (i = 0; i < N_STRIPS; i++) {
    if (strip_lightning_probs[i] > LIGHTNING_PROB_MIN + LIGHTNING_PROB_STEP) {
      (strip_lightning_probs[i]) -= LIGHTNING_PROB_STEP;
    } else {
      strip_lightning_probs[i] = LIGHTNING_PROB_MIN;
    }
  }

  for (i = 0; i < N_STRIPS; i++) {
    if (strip_lightning_states[i] == TRUE) {
      strip_lightning_states[i] = FALSE;
    } else {
      if ((uint8_t)rand() < strip_lightning_probs[i]) {
	strip_lightning_states[i] = TRUE;
      }
    }
  }

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }

    if (strip_lightning_states[strip_index] == TRUE) {
      r = 255;
      g = 255;
      b = 255;
    } else {
      r = 0;
      g = 0;
      b = 0;
    }
    
    matrix[x] = (r << 16) + (g << 8) + b;
  } 
}


static long strip_solid_colors[N_STRIPS];
static long strip_solid_darks[N_STRIPS];

static void InitSolidColors(void)
{
  int i, r, g, b;
  srand(TIMER_GetSysTick());
  for (i = 0; i < N_STRIPS; i++) {
    r = rand() & 0xff;
    g = rand() & 0xff;
    b = rand() & 0xff;
    r = (r * r) >> 8;
    g = (g * g) >> 8;
    b = (b * b) >> 8;
    strip_solid_colors[i] = (r << 16) + (g << 8) + b;
  }
}

static void InitSolidDarks(void)
{
  int i;
  long r, g, b;
  srand(TIMER_GetSysTick());
  for (i = 0; i < N_STRIPS; i++) {
    r = rand() & 0xff;
    g = rand() & 0xff;
    b = rand() & 0xff;
    r = (r * r * r * r) >> 24;
    g = (g * g * g * g) >> 24;
    b = (b * b * b * b) >> 24;
    strip_solid_darks[i] = (r << 16) + (g << 8) + b;
  }
}


static void SolidColorsStep(void)
{
  int strip_index, led_index;
  uint16_t  x, i, r, g, b;
  r = g = b = 0;

  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n\n", ev.code, ev.value);
    if (ev.value == 1 || ev.value == 2) { // key press
      CheckSceneChangeKeys(ev.code);     // check for scene change hotkey
    }
  }

  for (i = 0; i < N_STRIPS; i++) {
    if (motion_data[i + 1] != 0) {
      r = rand() & 0xff;
      g = rand() & 0xff;
      b = rand() & 0xff;
      r = (r * r) >> 8;
      g = (g * g) >> 8;
      b = (b * b) >> 8;
      strip_solid_colors[i] = (r << 16) + (g << 8) + b;
    }
  }
      

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }
    
    matrix[x] = strip_solid_colors[strip_index];
  } 
}

static void SolidDarksStep(void)
{
  int strip_index, led_index;
  uint16_t  x, i;
  long r, g, b;
  r = g = b = 0;

  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n\n", ev.code, ev.value);
    if (ev.value == 1 || ev.value == 2) { // key press
      CheckSceneChangeKeys(ev.code);     // check for scene change hotkey
    }
  }

  for (i = 0; i < N_STRIPS; i++) {
    if (motion_data[i + 1] != 0) {
      r = rand() & 0xff;
      g = rand() & 0xff;
      b = rand() & 0xff;
      r = (r * r * r * r) >> 24;
      g = (g * g * g * g) >> 24;
      b = (b * b * b * b) >> 24;
      strip_solid_darks[i] = (r << 16) + (g << 8) + b;
    }
  }
      

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }
    
    matrix[x] = strip_solid_darks[strip_index];
  } 
}

static void SolidAllStep(void)
{
  int strip_index, led_index;
  uint16_t  x, i;
  static long r = 25;
  static long g = 0;
  static long b = 5;

  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n\n", ev.code, ev.value);
    if (ev.value == 1 || ev.value == 2) { // key press
      CheckSceneChangeKeys(ev.code);     // check for scene change hotkey
    }
  }

  for (i = 0; i < N_STRIPS; i++) {
    if (motion_data[i + 1] != 0) {
      r = rand() & 0xff;
      g = rand() & 0xff;
      b = rand() & 0xff;
      r = (r * r * r * r) >> 24;
      g = (g * g * g * g) >> 24;
      b = (b * b * b * b) >> 24;
    }
  }
      

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }
    
    matrix[x] = (r << 16) + (g << 8) + b;
  } 
}

static void StaticStep(void)
{
  int strip_index, led_index;
  uint16_t  x, r, g, b, i;
  static uint16_t prob = 0x1fff;
  static uint16_t ran;
  r = g = b = 0;

  
  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n\n", ev.code, ev.value);
    if (ev.value == 1 || ev.value == 2) { // key press
      CheckSceneChangeKeys(ev.code);     // check for scene change hotkey
    }
  }

  if (scene != STATICS) {  // reinit for next entry
    prob = 0x1fff;   
  }

  for (i = 0; i < N_STRIPS; i++) {
    if (motion_data[i + 1] != 0) {
      if (prob < 0x7fff) {
	prob += 200;
      }
    }
  }

  if (prob > 0x00ff) {
    prob -= 40;
  }

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }

    ran = rand();
    if (ran < prob) {
      r = g = b = 255;
    } else {
      r = g = b = 0;
    }
    
    matrix[x] = (r << 16) + (g << 8) + b;
  } 
}

static void RGBFlashStep(void)
{
  int strip_index, led_index;
  uint16_t  x, r, g, b, i;
  static uint16_t t;
  static int flash_period = 10000;
  static int col = 0;
  r = g = b = 0;

  
  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n\n", ev.code, ev.value);
    if (ev.value == 1 || ev.value == 2) { // key press
      CheckSceneChangeKeys(ev.code);     // check for scene change hotkey
    }
  }

  for (i = 0; i < N_STRIPS; i++) {
    if (motion_data[i + 1] != 0) {
      if (flash_period > 100) {
	flash_period -= 100;
      }
    }
  }

  if (flash_period < 10000) {
    flash_period += 3;
  }
      

  t++;
  if (t + 4 >= flash_period / 100) {
    t = 0;
    col++;
    if (col == 3) {
      col = 0;
    }
  } 

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }

    r = 0;
    g = 0;
    b = 0;
    if (t == 0) {
      switch (col) {
      case 0:
	r = 255;
	break;
      case 1:
	g = 255;
	break;
      case 2:
	b = 255;
	break;
      }
    }
    
    matrix[x] = (r << 16) + (g << 8) + b;
  } 
}




static void StripLengthTestStep(void)
{
  int strip_index, led_index;
  uint16_t  i, x, r, g, b;
  static uint16_t t;
  static int active_strip;
  r = g = b = 0;
  t++;
  if (t == 100) {
    t = 0;
    /*  active_strip++;
    if (active_strip == N_STRIPS) {
      active_strip = 0;
      }*/
  }

  read(fd_key, &ev, sizeof(ev));
  if (ev.type == 1) {
    printf("key %i state %i\n", ev.code, ev.value);
    if (ev.value == 1) { // key press
      if (ev.code == 106) { // right arrow
	active_strip++;
	if (active_strip == N_STRIPS) {
	  active_strip = 0;
	}
      } else if (ev.code == 105) { // left arrow
	active_strip--;
	if (active_strip == -1) {
	  active_strip = N_STRIPS - 1;
	}
      } else if (ev.code == 103) { // up arrow
	(strip_lengths[active_strip])++;
      } else if (ev.code == 108) { // down arrow
	(strip_lengths[active_strip])--;
      } else if (ev.code == 25) { // p
	printf("strip lengths:\n{");
	for (i = 0; i < N_STRIPS; i++) {
	  printf("%i, ", strip_lengths[i]);
	}
	printf("};\n\n");
      }
    }
  }

    

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }

    if (strip_index == active_strip) {
      r = 255;
      g = 255;
      b = 255;
    } else {
      r = 0;
      g = 0;
      b = 0;
    }
    
    matrix[x] = (r << 16) + (g << 8) + b;
  }
}

static void XSweep(void)
{
  int strip_index, led_index;
  uint16_t  x, r, g, b;
  static uint16_t t;;
  r = g = b = 0;
  t++;
  if (t == 500) {
    t = -100;
    /*  active_strip++;
    if (active_strip == N_STRIPS) {
      active_strip = 0;
      }*/
  } 

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }

    if (strip_x[strip_index]*5 > t && strip_x[strip_index]*5 < t + 100) {
      r = 255;
      g = 255;
      b = 255;
    } else {
      r = 0;
      g = 0;
      b = 0;
    }
    
    matrix[x] = (r << 16) + (g << 8) + b;
  } 
}

static void YSweep(void)
{
   int strip_index, led_index;
  uint16_t  x, r, g, b;
  static uint16_t t;;
  r = g = b = 0;
  t++;
  if (t == 500) {
    t = -100;
    /*  active_strip++;
    if (active_strip == N_STRIPS) {
      active_strip = 0;
      }*/
  } 

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }

    if (strip_y[strip_index]*5 > t && strip_y[strip_index]*5 < t + 100) {
      r = 255;
      g = 255;
      b = 255;
    } else {
      r = 0;
      g = 0;
      b = 0;
    }
    
    matrix[x] = (r << 16) + (g << 8) + b;
  } 
}

static void ZSweep(void)
{
  int strip_index, led_index, strip_z;
  uint16_t  x, r, g, b;
  static uint16_t t;;
  r = g = b = 0;
  t++;
  if (t == 200) {
    t = -20;
    /*  active_strip++;
    if (active_strip == N_STRIPS) {
      active_strip = 0;
      }*/
  } 

  led_index = -1;
  strip_index = 0;
  for (x = 0; x < N_LEDS; x++) {
    led_index++;
    if (led_index == strip_lengths[strip_index]) {
      led_index = 0;
      strip_index++;
      if (strip_index == N_STRIPS) {
	break;
      }
    }
    strip_z = strip_lengths[strip_index] - led_index;

    if (strip_z*5 >= t && strip_z*5 < t + 20) {
      r = 255;
      g = 255;
      b = 255;
    } else {
      r = 0;
      g = 0;
      b = 0;
    }
    
    matrix[x] = (r << 16) + (g << 8) + b;
  } 
}
