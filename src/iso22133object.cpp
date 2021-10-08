#include <chrono>
#include <thread>

#include "iso22133object.hpp"
#include "iso22133state.hpp"
#include "iso22133.h"
#include "maestroTime.h"

#define TCP_BUFFER_SIZE 1024
#define UDP_BUFFER_SIZE 1024

namespace ISO22133 {
TestObject::TestObject(const std::string& listenIP)
	: name("myTestObject"),
	  ctrlChannel(BasicSocket::STREAM),
	  trajDecoder() {

	CartesianPosition initPos;
	SpeedType initSpd;
	AccelerationType initAcc;
	std::cout << "Listen IP: " << listenIP << std::endl;
	localIP = listenIP;
	initPos.isHeadingValid = false;
	initPos.isPositionValid = false;
	initSpd.isLateralValid = false;
	initSpd.isLongitudinalValid = false;
	initAcc.isLateralValid = false;
	initAcc.isLongitudinalValid = false;
	this->setPosition(initPos);
	this->setSpeed(initSpd);
	this->setAcceleration(initAcc);
	this->state = this->createInit();
	this->startHandleTCP();
	this->startHEABCheck();
	this->stateChangeSig.connect(&TestObject::onStateChange, this);
	this->osemSig.connect(&TestObject::onOSEM, this);
	this->heabSig.connect(&TestObject::onHEAB, this);
	this->ostmSig.connect(&TestObject::onOSTM, this);
	this->trajSig.connect(&TestObject::onTRAJ, this);
	this->strtSig.connect(&TestObject::onSTRT, this);
	this->heabTimeout.connect(&TestObject::onHeabTimeout, this);
}

TestObject::~TestObject() {
	on = false;
	try {
		heabMonrThread.join();
	} catch (std::system_error) {}
	try {
		tcpReceiveThread.join(); // This blocks forever when the receive thread has not finished accept()
	} catch (std::system_error) {}
	try {
		heabTimeoutThread.join();
	} catch (std::system_error) {}
}; 

void TestObject::disconnect() {
	ctrlChannel.close();
	processPort.close();
	heabMonrThread.join();
	awaitingFirstHeab = true;
}

void TestObject::receiveTCP() {
	std::cout << "Started TCP thread." << std::endl;
	while(this->on) {
		std::cout << "Awaiting TCP connection from server..." << std::endl;
		try {
			ctrlChannel = ctrlPort.await(
						localIP, ISO_22133_DEFAULT_OBJECT_TCP_PORT);
		} catch (std::exception& e) {
			std::cout << "pls fix" << std::endl;
			throw e;
		}
		std::cout << "TCP connection established." << std::endl;
		state->handleEvent(*this, ISO22133::Events::B);
		try {
			while (true) {
				auto data = ctrlChannel.recv();
				int nBytesHandled = 0;
				do {
					try {
						nBytesHandled = handleMessage(data);
					}
					catch (const std::exception& e) {
						std::cerr << e.what() << std::endl;
						break;
					}
					data.erase(data.begin(), data.begin() + nBytesHandled);
				} while(data.size() > 0);
			}
		} catch (SocketErrors::DisconnectedError&) {
			std::cout << "Connection to control center lost" << std::endl;
			state->handleEvent(*this, ISO22133::Events::L);
		}
	}
	std::cout << "Exiting TCP thread." << std::endl;
}

void TestObject::sendMONR(const BasicSocket::HostInfo& toWhere, bool debug) {
	std::vector<char> buffer(UDP_BUFFER_SIZE);
	struct timeval time;
	TimeSetToCurrentSystemTime(&time);
	auto result = encodeMONRMessage(&time, this->position, this->speed, this->acceleration,
									this->driveDirection, this->state->getStateID(), this->readyToArm,
									this->errorState, buffer.data(), buffer.size(),debug);
	if (result < 0) {
		std::cout << "Failed to encode MONR data" << std::endl;
	}
	else {
		processPort.sendto({buffer, toWhere}, static_cast<size_t>(result));
	}
}

void TestObject::heabMonrLoop() {
	std::cout << "Started MONR/HEAB communication thread." << std::endl;
	processPort.bind({localIP, ISO_22133_OBJECT_UDP_PORT});
	awaitingFirstHeab = true;
	std::cout << "Awaiting UDP data from server..." << std::endl;
	try {
		while (this->on) {
			auto [data, host] = processPort.recvfrom();
			if (awaitingFirstHeab) {
				std::cout << "Received UDP data from " << host.address << std::endl;
			}
			int nBytesHandled = 0;
			do {
				try {
					nBytesHandled = handleMessage(data);
					sendMONR(host);
				}
				catch (const std::exception& e) {
					std::cerr << e.what() << std::endl;
					break;
				}
				data.erase(data.begin(), data.begin() + nBytesHandled);
			} while(data.size() > 0);
		}
	} catch (SocketErrors::DisconnectedError&) {}
	std::cout << "Exiting MONR/HEAB communication thread." << std::endl;
}

void TestObject::checkHeabTimeout() {
	using namespace std::chrono;
	std::scoped_lock lock(heabMutex);
	// Check time difference of received HEAB and last HEAB
	auto timeSinceHeab = steady_clock::now() - lastHeabTime;
	if (!awaitingFirstHeab && timeSinceHeab > heartbeatTimeout) {
		std::cerr << "Heartbeat timeout: " << duration_cast<milliseconds>(timeSinceHeab).count()
				  << " ms since last heartbeat exceeds limit of " << heartbeatTimeout.count() << " ms."
				  << std::endl;
		heabTimeout();
	}
}

void TestObject::checkHeabLoop() {
	std::cout << "Started HEAB timeout thread." << std::endl;
	using namespace std::chrono;
	while (this->on) {
		auto t = std::chrono::steady_clock::now();
		checkHeabTimeout();
		// Don't lock the mutex all the time
		std::this_thread::sleep_until(t+expectedHeartbeatPeriod);
	}
	std::cout << "Exiting HEAB timeout thread." << std::endl;
}

void TestObject::onHeabTimeout() {
	this->state->handleEvent(*this, Events::L);
}

int TestObject::handleMessage(std::vector<char>& dataBuffer) {
	std::lock_guard<std::mutex> lock(this->recvMutex); // Both TCP and UDP threads end up in here
	int bytesHandled = 0;
	int debug = 0;
	struct timeval currentTime;
	TimeSetToCurrentSystemTime(&currentTime);

	ISOMessageID msgType = getISOMessageType(dataBuffer.data(), dataBuffer.size(), 0);
	// Ugly check here since we don't know if it is UDP or the rest of TRAJ
	if(msgType == MESSAGE_ID_INVALID && this->trajDecoder.ExpectingTrajPoints()) {
		msgType = MESSAGE_ID_TRAJ;
	}

	switch(msgType) {
	case MESSAGE_ID_TRAJ:
		try {
			bytesHandled = this->trajDecoder.DecodeTRAJ(dataBuffer);
		}
		catch(const std::exception& e) {
			throw e;
		}
		if(bytesHandled < 0) {
			throw std::invalid_argument("Error decoding TRAJ");
		};
		if (!this->trajDecoder.ExpectingTrajPoints()) {
			this->state->handleTRAJ(*this);
		}
		break;
	case MESSAGE_ID_OSEM:
		ObjectSettingsType OSEMstruct;
		bytesHandled = decodeOSEMMessage(
					&OSEMstruct,
					dataBuffer.data(),
					dataBuffer.size(),
					nullptr,
					debug);
		if(bytesHandled < 0) {
			throw std::invalid_argument("Error decoding OSEM");
		}
		std::cout << "Received OSEM " << std::endl;
		this->state->handleOSEM(*this, OSEMstruct);
		break;

	case MESSAGE_ID_OSTM:
		ObjectCommandType OSTMdata;
		bytesHandled = decodeOSTMMessage(
					dataBuffer.data(),
					dataBuffer.size(),
					&OSTMdata,
					debug);
		if(bytesHandled < 0) {
			throw std::invalid_argument("Error decoding OSTM");
		}
		this->state->handleOSTM(*this, OSTMdata);
		break;

	case MESSAGE_ID_STRT:
		StartMessageType STRTdata;
		bytesHandled = decodeSTRTMessage(
					dataBuffer.data(),
					dataBuffer.size(),
					&currentTime,
					&STRTdata,
					debug);
		if(bytesHandled < 0) {
			throw std::invalid_argument("Error decoding STRT");
		}
		this->state->handleSTRT(*this, STRTdata);
		break;

	case MESSAGE_ID_HEAB:
		HeabMessageDataType HEABdata;

		bytesHandled = decodeHEABMessage(
					dataBuffer.data(),
					dataBuffer.size(),
					currentTime,
					&HEABdata,
					debug);
		if(bytesHandled < 0) {
			throw std::invalid_argument("Error decoding HEAB");
		}
		this->handleHEAB(HEABdata);
		break;
	default:
		bytesHandled = static_cast<int>(dataBuffer.size());
		break;
	}

	return bytesHandled;
}

/**
 * @brief Generates state changes based on control center status
 * @param heab struct HeabMessageDataType
 */
void TestObject::handleHEAB(HeabMessageDataType& heab) {
	using namespace std::chrono;
	// Order matters here, below may change state
	// causing the signal to not be triggered if placed
	// after the handleEvent() calls
	heabSig(heab);

	// Check network delay: difference between
	// timestamp in HEAB and local time
	auto heabTime = seconds(heab.dataTimestamp.tv_sec)
			+ microseconds(heab.dataTimestamp.tv_usec);
	auto networkDelay = system_clock::now().time_since_epoch() - heabTime;
	if (networkDelay > maxSafeNetworkDelay) {
		std::cerr << "Network delay of " << duration_cast<milliseconds>(networkDelay).count()
				  << " ms exceeds safe limit of " << maxSafeNetworkDelay.count() << " ms."
				  << std::endl;
		// TODO do something
	}
	checkHeabTimeout();
	std::scoped_lock lock(heabMutex);
	lastHeabTime = steady_clock::now();
	awaitingFirstHeab = false;

	switch (heab.controlCenterStatus) {
	case CONTROL_CENTER_STATUS_NORMAL_STOP:
		this->state->handleEvent(*this, ISO22133::Events::U);
		break;
	case CONTROL_CENTER_STATUS_ABORT:
		this->state->handleEvent(*this, ISO22133::Events::W);
		break;
	case CONTROL_CENTER_STATUS_TEST_DONE:
		this->state->handleEvent(*this, ISO22133::Events::Y);
		break;
	default:
		break;
	}
	ccStatus = heab.controlCenterStatus;
	return;
}

} //namespace ISO22133
