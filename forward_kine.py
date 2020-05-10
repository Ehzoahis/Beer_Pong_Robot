import numpy as np
import scipy.linalg as la

def get_v(w, q):
	return -1*np.cross(w.T ,q.T).T

def Get_MS(X, Y, Z, endor):
	S = np.zeros((6,6))
	M = np.array([[0,0,-1,endor[0]],[1,0,0,endor[1]],[0,-1,0,endor[2]],[0,0,0,1]])
	w1 = np.array([[0],[0],[1]])
	w2 = np.array([[-1],[0],[0]])
	w3 = np.array([[-1],[0],[0]])
	w4 = np.array([[-1],[0],[0]])
	w5 = np.array([[0],[1],[0]])
	w6 = np.array([[-1],[0],[0]])
	q1 = np.array([[X[0]],[Y[0]],[Z[0]]])
	q2 = np.array([[X[1]],[Y[1]],[Z[1]]])
	q3 = np.array([[X[2]],[Y[2]],[Z[2]]])
	q4 = np.array([[X[3]],[Y[3]],[Z[3]]])
	q5 = np.array([[X[4]],[Y[4]],[Z[4]]])
	q6 = np.array([[X[5]],[Y[5]],[Z[5]]])

	W = np.array([w1, w2, w3, w4, w5, w6])
	Q = np.array([q1, q2, q3, q4, q5, q6])
	
	V = []
	
	for i in range(6):
		V.append(get_v(W[i], Q[i]))
	V = np.array(V)

	for i in range(6):
		S[:,i] = np.concatenate((W[i], V[i]), axis=0).T

	return M, S


def get_T(theta, M, S):
	PoE = 1

	for i in range(6):
		w = S[:3,i]
		v = S[3:,i]
		w1 = w[0]
		w2 = w[1]
		w3 = w[2]
		w_bracket = np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]])	

		t = np.concatenate((w_bracket, v.reshape(3,1)), axis=1)
		b = np.array([[0, 0, 0, 0]])
		s = np.concatenate((t,b), axis=0)
		PoE = np.dot(PoE, la.expm(s * theta[i]))

	T = np.round(np.dot(PoE, M), 3)
	return T