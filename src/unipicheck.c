
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <regex.h>
#include <sys/wait.h>

#include "unipiutil.h"

#define MAXTARGET 100
int calltarget(char* target)
{
	char ctarget[MAXTARGET];
	pid_t cpid;

	memset(ctarget, 0, MAXTARGET);
	strncpy(ctarget, target, MAXTARGET-1);
	printf("  >>>> %s\n", ctarget);
	cpid = fork();
	if (cpid == -1) {
		printf("Error forking process\n");
		return -1;
	}
    if (cpid == 0){
		return execl("/bin/systemctl", "--no-block", "start", ctarget, NULL);
	}
	return 0;
}

int is_valid_char(char ch)
{
	if (ch == '.') return 1;
	if (ch == '_') return 1;
	if (ch == '-') return 1;
	if ((ch >= '0') && (ch <= '9')) return 1;
	if ((ch >= 'a') && (ch <= 'z')) return 1;
	if ((ch >= 'A') && (ch <= 'Z')) return 1;
	return 0;
}

void split_targets(char* targets)
{
	char* token;
	while (1) {
		/* skip blank chars */ 
		while ((targets[0] == ' ') || (targets[0] == '\t')) targets++;  
		/* end of line ? */ 
		if ((targets[0] == '\0') || (targets[0] == '\n')) return;

		token = targets;
		while (is_valid_char(targets[0])) targets++;
		/* check delimiter; if not valid then exit */
		if ((targets[0] != ' ') && (targets[0] != '\t') && (targets[0] != '\0') && (targets[0] != '\n')) return;
		calltarget(token);
	}
}

int match(char* regstring, char* model)
{
	regex_t regex;
	int i;
	/* strip blanks in regstring */
	while ((regstring[0] == ' ') || (regstring[0] == '\t')) regstring++;
	for (i = strlen(regstring)-1; i >= 0; i--) {
		if ((regstring[i] != ' ') && (regstring[i] != '\t')) {
			break;
		}
		regstring[i] = '\0';
	}
	//printf("--%s-- --%s--\n", regstring, model);
	int reti = regcomp(&regex, regstring, 0);
	if (reti) return 0;
	reti = regexec(&regex, model, 0, NULL, 0);
	return (!reti);
}


#define MAXLINE 1024
int main(int argc, char** argv)
{
   int res;
   int i;
   ssize_t n;
   ssize_t alloc = MAXLINE;
   char *unipi_model;
   FILE* ftable;
   char *saveptr;
   char *line, *token;

   unipi_model = get_unipi_name();
   if (unipi_model[0]=='\0') return 0;

   /* try to open */
   ftable = fopen("/opt/unipi/data/unipi-target.map", "r");
   if (ftable == NULL) {
		/* fail-safe behavior */
		if (strcmp(unipi_model,"UNIPI1") == 0) {
			calltarget("unipi1.target");
		} else {
			calltarget("unipispi.target");
		}
		while (waitpid(-1, NULL, 0) > 0);
		return 0;
   }

   line = malloc(alloc);
   n = getline(&line, &alloc, ftable);
   while (n >= 0) {
        token = strtok_r(line, "=", &saveptr);
		if (token != NULL) {
			if (match(token, unipi_model)) {
        		token = strtok_r(NULL, "\n", &saveptr);
				if (token != NULL) {
					split_targets(token);
				}
			}
		}

   		n = getline(&line, &alloc, ftable);
   }

   free(line);
   fclose(ftable);
   while (waitpid(-1, NULL, 0) > 0);

   return 0;
}
