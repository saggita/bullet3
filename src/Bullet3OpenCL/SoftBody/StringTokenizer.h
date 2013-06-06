#include <string>
#include <vector>

using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

int StringTokenizer(const string& input, const string& delimiter, 
				vector<string>& results, bool includeEmpties);


#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
