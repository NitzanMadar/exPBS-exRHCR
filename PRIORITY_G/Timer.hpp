#pragma once

#include <chrono>
#include <iostream>

class ScopedTimer
{
public:
	ScopedTimer()
		: start_(std::chrono::high_resolution_clock::now())
	{
	}

	double elapsedSeconds() const {
		auto end = std::chrono::high_resolution_clock::now();
		auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(end - start_);
		return timeSpan.count();
	}

	~ScopedTimer()
	{
		std::cout << "Elapsed: " << elapsedSeconds() << " s" << std::endl;
	}

private:
	std::chrono::high_resolution_clock::time_point start_;
};
