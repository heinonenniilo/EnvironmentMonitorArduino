#pragma once

void recordCommunicationError();
void clearCommunicationErrors();

void handleCommand(const char* command);
void reportBrokerCommunicationError();
void setStatus(int statusToSet);
