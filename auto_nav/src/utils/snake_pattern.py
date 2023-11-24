# Python 3 program to print
# matrix in snake order
M = 4
N = 4


def printf(mat):
	global M, N

	# Traverse through all rows
	for i in range(M):

		# If current row is
		# even, print from
		# left to right
		if i % 2 == 0:
			for j in range(N):
				print(str(mat[i][j]),
					end=" ")

		# If current row is
		# odd, print from
		# right to left
		else:
			for j in range(N - 1, -1, -1):
				print(str(mat[i][j]),
					end=" ")


# Driver code
mat = [[10, 20, 30, 40],
	[15, 25, 35, 45],
	[27, 29, 37, 48],
	[32, 33, 39, 50]]

printf(mat)

# This code is contributed
# by ChitraNayal
