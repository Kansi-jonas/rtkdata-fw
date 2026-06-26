#pragma once

// Captive-portal DNS hijack. While running, answers EVERY DNS A query on the
// SoftAP with the AP's own IP, so a client that joins the setup AP triggers its
// OS "sign in to network" popup (the OS connectivity-check hostname resolves to
// us and hits the web server's redirect to "/"). Setup-only; start it when the
// AP comes up and stop it when the AP goes down. Safe to call repeatedly.
void dns_hijack_start(void);
void dns_hijack_stop(void);
