#pragma once

void recordCommunicationError();
void clearCommunicationErrors();
void reportCommunicationError();

void handleCommand(const char* command);
void setStatus(int statusToSet);
