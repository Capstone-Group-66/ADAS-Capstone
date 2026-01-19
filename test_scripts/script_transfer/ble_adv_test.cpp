// ble_adv_test.cpp
// Minimal BLE advertiser using BlueZ HCI API.
// Goal: make the Jetson show up as "JetsonBLETest" in a phone BLE scanner.

#include <iostream>
#include <cstring>
#include <cstdlib>

extern "C" {
#include <unistd.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
}

static int send_hci_cmd(int dd,
                        uint16_t ocf,
                        uint8_t clen,
                        void *cparam,
                        uint8_t rlen,
                        void *rparam)
{
    struct hci_request rq;
    std::memset(&rq, 0, sizeof(rq));
    rq.ogf   = OGF_LE_CTL;
    rq.ocf   = ocf;
    rq.cparam = cparam;
    rq.clen   = clen;
    rq.rparam = rparam;
    rq.rlen   = rlen;

    if (hci_send_req(dd, &rq, 1000) < 0) {
        perror("hci_send_req");
        return -1;
    }
    return 0;
}

int main() {
    int dev_id = hci_get_route(nullptr);
    if (dev_id < 0) {
        std::cerr << "No Bluetooth adapter found (hci_get_route failed).\n";
        return 1;
    }

    int dd = hci_open_dev(dev_id);
    if (dd < 0) {
        perror("hci_open_dev");
        return 1;
    }

    uint8_t status = 0;

    // 1) Set advertising parameters
    le_set_advertising_parameters_cp adv_params;
    std::memset(&adv_params, 0, sizeof(adv_params));

    // 100 ms advertising interval
    adv_params.min_interval = htobs(0x00A0);
    adv_params.max_interval = htobs(0x00A0);

    adv_params.advtype          = 0x00; // ADV_IND (connectable undirected)
    adv_params.own_bdaddr_type  = 0x00; // public address
    adv_params.direct_bdaddr_type = 0x00;
    std::memset(&adv_params.direct_bdaddr, 0, sizeof(adv_params.direct_bdaddr));
    adv_params.chan_map         = 0x07; // channels 37,38,39
    adv_params.filter           = 0x00; // allow any

    if (send_hci_cmd(dd,
                     OCF_LE_SET_ADVERTISING_PARAMETERS,
                     LE_SET_ADVERTISING_PARAMETERS_CP_SIZE,
                     &adv_params,
                     1,
                     &status) < 0) {
        std::cerr << "Failed to set advertising parameters.\n";
        hci_close_dev(dd);
        return 1;
    }

    if (status != 0x00) {
        std::cerr << "Controller reported error for adv params, status = "
                  << static_cast<int>(status) << "\n";
        hci_close_dev(dd);
        return 1;
    }

    // 2) Set advertising data (flags + complete local name)
    le_set_advertising_data_cp adv_data_cp;
    std::memset(&adv_data_cp, 0, sizeof(adv_data_cp));

    uint8_t adv_data[31];
    int len = 0;

    // Flags: LE General Discoverable, BR/EDR Not Supported
    adv_data[len++] = 2;      // length
    adv_data[len++] = 0x01;   // Flags type
    adv_data[len++] = 0x06;   // Flags value

    // Complete Local Name
    const char *name = "JetsonBLETest";
    size_t name_len = std::strlen(name);
    if (name_len > 29) name_len = 29; // safety

    adv_data[len++] = static_cast<uint8_t>(name_len + 1); // length of this AD structure
    adv_data[len++] = 0x09;   // Complete Local Name type
    std::memcpy(&adv_data[len], name, name_len);
    len += static_cast<int>(name_len);

    adv_data_cp.length = static_cast<uint8_t>(len);
    std::memcpy(adv_data_cp.data, adv_data, len);

    if (send_hci_cmd(dd,
                     OCF_LE_SET_ADVERTISING_DATA,
                     LE_SET_ADVERTISING_DATA_CP_SIZE,
                     &adv_data_cp,
                     1,
                     &status) < 0) {
        std::cerr << "Failed to set advertising data.\n";
        hci_close_dev(dd);
        return 1;
    }

    if (status != 0x00) {
        std::cerr << "Controller reported error for adv data, status = "
                  << static_cast<int>(status) << "\n";
        hci_close_dev(dd);
        return 1;
    }

    // 3) Enable advertising
    le_set_advertise_enable_cp enable_cp;
    std::memset(&enable_cp, 0, sizeof(enable_cp));
    enable_cp.enable = 0x01; // enable

    if (send_hci_cmd(dd,
                     OCF_LE_SET_ADVERTISE_ENABLE,
                     LE_SET_ADVERTISE_ENABLE_CP_SIZE,
                     &enable_cp,
                     1,
                     &status) < 0) {
        std::cerr << "Failed to enable advertising.\n";
        hci_close_dev(dd);
        return 1;
    }

    if (status != 0x00) {
        std::cerr << "Controller reported error enabling advertising, status = "
                  << static_cast<int>(status) << "\n";
        hci_close_dev(dd);
        return 1;
    }

    std::cout << "Now advertising as \"JetsonBLETest\".\n";
    std::cout << "Press Enter to stop advertising and exit.\n";
    std::cin.get();

    // 4) Disable advertising
    enable_cp.enable = 0x00; // disable
    if (send_hci_cmd(dd,
                     OCF_LE_SET_ADVERTISE_ENABLE,
                     LE_SET_ADVERTISE_ENABLE_CP_SIZE,
                     &enable_cp,
                     1,
                     &status) < 0) {
        std::cerr << "Failed to disable advertising.\n";
    }

    hci_close_dev(dd);
    return 0;
}

