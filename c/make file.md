- A phony target in a Makefile is a target that does not correspond to an actual file but instead runs commands. To define one, use the .PHONY directive.
```bash
#example
.PHONY: clean
clean:
	rm -rf *.o my_program
```
how to run
```bash
make clean
```

- ifneq (var_1,var_2)
	- this will check var_1 not equal to var_2
	- if true execute some statements in body of it