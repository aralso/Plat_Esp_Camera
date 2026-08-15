#include "lpc.h"

#include <chrono>

enum LPC_MARKER
{
	ENCODE_FRAME,
	DECODE_JPEG,

	DO_ENCODE,
	SELECT_MODE,
	BUILD_RESIDUALS,
	CBP_FLAGS,

	ENCODE_MB,
	ENCODE_RESIDUAL_4x4,
	ENCODE_RESIDUAL_16x16,
	ENCODE_RESIDUAL_CHROMA,
	CABAC_ENCODE,
	CABAC_BYPASS,

	ADD_RESIDUALS,
	NEIGHBOUR_UPDATE,

	LPC_MARKER_COUNT
};

static const char* to_string(LPC_MARKER id)
{
	switch (id)
	{
	case ENCODE_FRAME:
		return "ENCODE_FRAME";
	case DECODE_JPEG:
		return "DECODE_JPEG";

	case DO_ENCODE:
		return "DO_ENCODE";
	case SELECT_MODE:
		return "SELECT_MODE";
	case BUILD_RESIDUALS:
		return "BUILD_RESIDUALS";
	case CBP_FLAGS:
		return "CBP_FLAGS";

	case ENCODE_MB:
		return "ENCODE_MB";
	case ENCODE_RESIDUAL_4x4:
		return "ENCODE_RESIDUAL_4x4";
	case ENCODE_RESIDUAL_16x16:
		return "ENCODE_RESIDUAL_16x16";
	case ENCODE_RESIDUAL_CHROMA:
		return "ENCODE_RESIDUAL_CHROMA";
	case CABAC_ENCODE:
		return "CABAC_ENCODE";
	case CABAC_BYPASS:
		return "CABAC_BYPASS";

	case ADD_RESIDUALS:
		return "ADD_RESIDUALS";
	case NEIGHBOUR_UPDATE:
		return "NEIGHBOUR_UPDATE";

	default:
		return "(unknown)";
	};
}

struct lpc_profiler_t
{
	struct stats_t
	{
		uint64_t count;
		uint64_t total;
		uint64_t min;
		uint64_t max;
		uint64_t nesting;
	};

	static stats_t markers[LPC_MARKER_COUNT];
	static int nesting;

	explicit lpc_profiler_t(LPC_MARKER _id)
	{
		if (_id == ENCODE_FRAME)
			lpc_profiler_t::reset();

		id = _id;
		start = Clock::now();
		nesting++;
	}

	~lpc_profiler_t()
	{
		nesting--;
		uint64_t elapsed =
			std::chrono::duration_cast<std::chrono::nanoseconds>(
				Clock::now() - start).count();

		stats_t& stat = markers[id];

		stat.count++;
		stat.total += elapsed;

		if (elapsed < stat.min)
			stat.min = elapsed;

		if (elapsed > stat.max)
			stat.max = elapsed;

		if (nesting > stat.nesting)
			stat.nesting = nesting;
	}

	LPC_DEBUG_ONLY(static void print());

private:
	static void reset()
	{
		nesting = 0;
		for (int i = 0; i < LPC_MARKER_COUNT; i++)
		{
			stats_t& s = markers[i];

			s.count = 0;
			s.total = 0;
			s.min = UINT64_MAX;
			s.max = 0;
			s.nesting = 0;
		}
	}

	typedef std::chrono::steady_clock Clock;

	LPC_MARKER id;
	Clock::time_point start;
};
