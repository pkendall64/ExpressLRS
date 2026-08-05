#include "TXOTAConnector.h"

#include "common.h"
#include "config.h"
#include "options.h"
#include "stubborn_sender.h"
#include "FIFO.h"
#include "telemetry_protocol.h"

extern StubbornSender DataUlSender;

namespace
{
constexpr auto MSP_SERIAL_OUT_FIFO_SIZE = 256U;
FIFO<MSP_SERIAL_OUT_FIFO_SIZE> outputQueue;
uint8_t currentTransmissionBuffer[ELRS_DATA_UL_BUFFER] = {};
uint8_t currentTransmissionLength = 0;

void unlockMessage()
{
    // the current msp message is sent so restore the next buffered write
    if (outputQueue.size() > 0)
    {
        outputQueue.lock();
        currentTransmissionLength = outputQueue.pop();
        outputQueue.popBytes(currentTransmissionBuffer, currentTransmissionLength);
        outputQueue.unlock();
    }
    else
    {
        // no msp message is ready to send currently
        currentTransmissionLength = 0;
    }
}

void pumpSender()
{
    static bool transferActive = false;
    // sending is done and we need to update our flag
    if (transferActive)
    {
        // unlock buffer for msp messages
        unlockMessage();
        transferActive = false;
    }
    // we are not sending so look for next msp package
    if (!transferActive)
    {
        // if we have a new msp package start sending
        if (currentTransmissionLength > 0)
        {
            DataUlSender.SetDataToTransmit(currentTransmissionBuffer, currentTransmissionLength);
            transferActive = true;
        }
    }
}

void resetOutputQueue()
{
    outputQueue.flush();
    currentTransmissionLength = 0;
}

bool initialize()
{
    return !firmwareOptions.is_airport;
}

int start()
{
    if (!InBindingMode && config.GetLinkMode() != TX_MAVLINK_MODE) return DURATION_IMMEDIATELY;
    return DURATION_NEVER;
}

int event()
{
    static connectionState_e lastConnectionState = noCrossfire;
    if (lastConnectionState != connected && connectionState == connected)
    {
        resetOutputQueue();
    }
    lastConnectionState = connectionState;
    return start();
}

int timeout()
{
    pumpSender();
    return DURATION_IMMEDIATELY;
}
}


TXOTAConnector::TXOTAConnector()
{
    // add the devices that we know are reachable via this connector
    addDevice(CRSF_ADDRESS_CRSF_RECEIVER);
    addDevice(CRSF_ADDRESS_FLIGHT_CONTROLLER);
}

void TXOTAConnector::forwardMessage(const crsf_header_t *message)
{
    if (connectionState == connected)
    {
        const uint8_t length = message->frame_size + 2;
        if (length > ELRS_DATA_UL_BUFFER)
        {
            return;
        }

        // store next msp message
        const auto data = (uint8_t *)message;
        if (currentTransmissionLength == 0)
        {
            for (uint8_t i = 0; i < length; i++)
            {
                currentTransmissionBuffer[i] = data[i];
            }
            currentTransmissionLength = length;
        }
        // store all write-requests since an update does send multiple writes
        else
        {
            outputQueue.lock();
            if (outputQueue.ensure(length + 1))
            {
                outputQueue.push(length);
                outputQueue.pushBytes((const uint8_t *)data, length);
            }
            outputQueue.unlock();
        }
    }
}

device_t CRSFUplink_device {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_CONNECTION_CHANGED | EVENT_CONFIG_MAIN_CHANGED | EVENT_ENTER_BIND_MODE | EVENT_EXIT_BIND_MODE,
};
