# Kayacan et al. (2012) 논문 정리
*Modeling and Control of a Spherical Rolling Robot: A Decoupled Dynamics Approach*

---

## 1. 좌표계 (Reference Frames)

논문에서는 세 개의 좌표계를 정의한다.

![[Pasted image 20260403033351.png]]

### $R_f$ : $O - XYZ$ (관성 좌표계)

- 지면에 고정된 **관성 좌표계(inertial frame)** 이다.
- 모든 운동의 기준이 되는 절대 좌표계이다.
- 구의 중심 위치 $(x, y)$와 오일러 각도 $\delta_1, \delta_2, \delta_3$가 이 프레임 기준으로 정의된다.
  *(오일러 각도 $\delta_1, \delta_2, \delta_3$는 논문의 2.1절에서만 사용되며 이후 다시 등장하지 않음. 다른 논문에서 차용한 것으로 Kayacan 논문 고유의 표기가 아님. 헷갈리지 말기.)*

### $R_{f_0}$ : $O_0 - X_0Y_0Z_0$ (구 중심 부착, 병진만 허용)

- 구의 중심에 붙어서 **구와 함께 이동(translate)** 하지만, **회전은 $R_f$와 동일**하다.
- $R_f$에 대해 **평행이동만** 허용되고 회전은 없다.
- 진자의 위치벡터 $\mathbf{r}^{p_0}$와 각속도 $\boldsymbol{\omega}^{p_0}$가 이 프레임에서 표현된다.
- 관성계와 회전 방향이 같으므로, 사실상 "구 중심을 원점으로 한 관성계"처럼 동작한다.

### $R_{f_1}$ : $O_1 - X_1Y_1Z_1$ (구 중심 부착, 회전만 허용)

- 역시 구의 중심에 붙어 있지만, $R_{f_0}$에 대해 **회전만** 허용된다.
- 구의 회전 상태를 따라가는 **body frame**이다.
- 축 정의:
  - $O - x_1$ : 횡방향 축 (transversal axis)
  - $O - y_1$ : 종방향 축 (longitudinal axis)
- $R_{f_0}$의 x축을 기준으로 CW 방향으로 $\theta$만큼, y축을 기준으로 CW 방향으로 $\phi$만큼 회전하여 $R_{f_1}$이 만들어진다.
- 논문의 $T^0_{1x},\ T^0_{1y}$는 $R_{f_1}$에서 표현된 벡터를 $R_{f_0}$ 기준으로 변환하는 행렬이다 ($R_{f_1} \to R_{f_0}$). 위 CW 회전의 역변환이므로 수치적으로 CCW 회전행렬 형태를 띤다 (논문 식 (3), (4)):


$$T^0_{1x}(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\theta & \sin\theta \\ 0 & -\sin\theta & \cos\theta \end{bmatrix}, \qquad T^0_{1y}(\phi) = \begin{bmatrix} \cos\phi & 0 & -\sin\phi \\ 0 & 1 & 0 \\ \sin\phi & 0 & \cos\phi \end{bmatrix}$$


### 세 좌표계의 관계

$$R_f \xrightarrow{\text{평행이동만}} R_{f_0} \xrightarrow{\text{회전만}} R_{f_1}$$

$R_{f_0}$와 $R_{f_1}$을 분리한 이유는 구의 **병진 운동**과 **회전 운동**을 명확히 분리하기 위함이다.

---

## 2. Kinematic Model

### 2.1 구 중심의 속도 표현

#### 단위벡터 $\mathbf{i}, \mathbf{j}, \mathbf{k}$의 프레임

$\mathbf{i}, \mathbf{j}, \mathbf{k}$는 $R_{f_0}$의 단위벡터이다. $R_{f_0}$는 병진만 허용되고 회전은 없으므로, 방향은 항상 관성계 $R_f$와 동일하다.

- $\mathbf{i}$ : $x_0$축 방향 (횡방향, transversal axis)
- $\mathbf{j}$ : $y_0$축 방향 (종방향, longitudinal axis)
- $\mathbf{k}$ : $z_0$축 방향 (연직 상방)

![[Pasted image 20260403032325.png]]
*그림을 보면 $\mathbf{i}, \mathbf{j}, \mathbf{k}$가 지면에 대해 기울어진 것처럼 보이지만 $\mathbf{i}, \mathbf{j}, \mathbf{k}$는 $R_{f_0}$의 좌표축이므로 지면에 대해 기울어지지 않음. 논문이 $R_{f_0}$의 좌표축과 $R_{f_1}$의 좌표축을 모두 $\mathbf{i}, \mathbf{j}, \mathbf{k}$로 혼용하여 표기하므로 주의.*

#### 각속도

구가 $x$축 주위로 각도 $\theta$만큼 굴러가는 경우, 오른손 법칙 기준으로 구가 $+y$ 방향으로 전진하는 것은 $x$축에 대한 음의 방향 회전에 해당한다. 논문에서 $\theta$는 전진 방향으로 굴러간 각도로 정의되므로:

$$\omega^s_x = -\dot{\theta}\,\mathbf{i}$$

$$\omega^s_y = -\dot{\phi}\,\mathbf{j}$$

#### 선속도 유도: 구르기 조건 적용

선속도는 **구르기 조건(rolling without slipping)** 으로부터 유도된다. 접촉점 $P$의 속도는 0이어야 하므로:

$$\mathbf{v}_P = \mathbf{v}_{\text{center}} + \boldsymbol{\omega} \times \mathbf{r}_{O \to P} = \mathbf{0}$$

구 중심 $O$에서 접촉점 $P$까지의 벡터는 $\mathbf{r}_{O \to P} = -R\mathbf{k}$이므로, 정리하면:

$$\mathbf{v}_{\text{center}} = \boldsymbol{\omega} \times R\mathbf{k}$$

**x-subsystem** ($\boldsymbol{\omega} = -\dot{\theta}\,\mathbf{i}$ 대입):

$$\mathbf{v}^s_x = \left(-\dot{\theta}\,\mathbf{i}\right) \times R\mathbf{k} = -R\dot{\theta}\left(\mathbf{i} \times \mathbf{k}\right)$$

$\mathbf{i} \times \mathbf{k} = -(\mathbf{k} \times \mathbf{i}) = -\mathbf{j}$ 이므로:

$$\boxed{\mathbf{v}^s_x = -R\dot{\theta}\,\mathbf{j}}$$

**y-subsystem** ($\boldsymbol{\omega} = -\dot{\phi}\,\mathbf{j}$ 대입):

$$\mathbf{v}^s_y = \left(-\dot{\phi}\,\mathbf{j}\right) \times R\mathbf{k} = -R\dot{\phi}\left(\mathbf{j} \times \mathbf{k}\right) = -R\dot{\phi}\,\mathbf{i}$$

$$\boxed{\mathbf{v}^s_y = R\dot{\phi}\,\mathbf{i}}$$

두 경우 모두 크기는 각각 $R|\dot{\theta}|$, $R|\dot{\phi}|$이며 방향만 convention에 따른다.

---

### 2.2 진자 질량 중심의 위치벡터 표현

#### i, j, k 혼용 문제

논문은 $R_{f_1}$ 기준 진자 위치(식 (9), (10))와 $R_{f_0}$ 기준으로 변환한 결과(식 (11), (12)) 모두를 동일한 기호 $\mathbf{i}, \mathbf{j}, \mathbf{k}$로 표기한다. 두 프레임이 서로 회전되어 있기 때문에 물리적으로 다른 방향을 가리키지만 논문이 이를 구분하지 않으므로 주의가 필요하다.

- 식 (9), (10)의 $\mathbf{i}, \mathbf{j}, \mathbf{k}$ → **$R_{f_1}$의 기저벡터** (구와 함께 회전하는 body frame 기준)
- 식 (11), (12)의 $\mathbf{i}, \mathbf{j}, \mathbf{k}$ → **$R_{f_0}$의 기저벡터** (관성계와 방향이 같은 frame 기준)

#### $R_{f_1}$ 기준 진자 위치 (논문 식 (9), (10))

진자는 구 중심에 매달려 있으며 정적 평형 상태에서 연직 아래를 향한다. $\mathbf{k}$가 위를 향하므로 평형 위치는 $[0,\ 0,\ -l]^T$이다.

![[Pasted image 20260403143312.png]]


**x-subsystem**: 평형 위치를 x축 주위로 $\alpha$만큼 회전하면:

$$\begin{bmatrix}1&0&0\\0&\cos\alpha&-\sin\alpha\\0&\sin\alpha&\cos\alpha\end{bmatrix} \begin{bmatrix}0\\0\\-l\end{bmatrix} = \begin{bmatrix}0\\l\sin\alpha\\-l\cos\alpha\end{bmatrix}$$

$$\mathbf{r}^{p_1}_x = l\sin\alpha\,\mathbf{j} - l\cos\alpha\,\mathbf{k} \tag{9}$$

**y-subsystem**: 평형 위치를 y축 주위로 $\beta$만큼 회전하면:

$$\begin{bmatrix}\cos\beta&0&\sin\beta\\0&1&0\\-\sin\beta&0&\cos\beta\end{bmatrix} \begin{bmatrix}0\\0\\-l\end{bmatrix} = \begin{bmatrix}-l\sin\beta\\0\\-l\cos\beta\end{bmatrix}$$

$$\mathbf{r}^{p_1}_y = -l\sin\beta\,\mathbf{i} - l\cos\beta\,\mathbf{k} \tag{10}$$

#### $R_{f_0}$ 기준 진자 위치 (논문 식 (11), (12))

$T^0_{1x}$를 곱하여 $R_{f_1}$ 기준 좌표를 $R_{f_0}$ 기준으로 변환한다.

**x-subsystem**:

$$\mathbf{r}^{p_0}_x = T^0_{1x}(\theta)\,\mathbf{r}^{p_1}_x = \begin{bmatrix}1&0&0\\0&\cos\theta&\sin\theta\\0&-\sin\theta&\cos\theta\end{bmatrix}\begin{bmatrix}0\\l\sin\alpha\\-l\cos\alpha\end{bmatrix} = \begin{bmatrix}0\\l\sin(\alpha-\theta)\\-l\cos(\alpha-\theta)\end{bmatrix}$$

$$\mathbf{r}^{p_0}_x = l\sin(\alpha-\theta)\,\mathbf{j} - l\cos(\alpha-\theta)\,\mathbf{k} \tag{11}$$

**y-subsystem**:

$$\mathbf{r}^{p_0}_y = T^0_{1y}(\phi)\,\mathbf{r}^{p_1}_y = -l\sin(\beta-\phi)\,\mathbf{i} - l\cos(\beta-\phi)\,\mathbf{k} \tag{12}$$

#### 결과의 물리적 의미

$(\alpha - \theta)$가 나타나는 이유: $R_{f_1}$은 구가 $\theta$만큼 굴러가면서 함께 회전한 frame이다. 진자는 $R_{f_1}$ 기준으로 $\alpha$만큼 기울어져 있으므로, 지면에 고정된 $R_{f_0}$ 기준으로 보면 진자의 실제 기울기는 $(\alpha - \theta)$가 된다. 변환행렬 계산이 이 물리적 의미를 정확히 담아낸다.

마지막으로 논문이 "$R_f$에서 정의된 위치벡터 $\mathbf{r}^p$는 $\mathbf{r}^{p_0}$와 동일하다"고 명시하는 이유는, $R_{f_0}$가 $R_f$와 방향이 같고 병진만 다르기 때문에 벡터의 성분값 자체는 동일하기 때문이다.

---
### 2.3 진자의 각속도 및 선속도 표현

#### 진자의 각속도 (논문 식 13-16)

**$R_{f_1}$ 기준 진자 각속도**

진자가 $R_{f_1}$의 x축 주위로 각도 $\alpha$만큼 회전하고 있으므로, $R_{f_1}$ 기준으로는 단순히:

$$\omega^{p_1}_x = \dot{\alpha}\,\mathbf{i} \tag{13}$$

$$\omega^{p_1}_y = \dot{\beta}\,\mathbf{j} \tag{15}$$

**$R_{f_0}$ 기준 진자 각속도**

$R_{f_0}$ 기준 진자의 절대 각속도는 **구의 각속도 + 진자의 상대 각속도**로 구성된다:

$$\omega^{p_0}_x = \omega^s_x + T^0_{1x}\,\omega^{p_1}_x$$

여기서 $\omega^{p_1}_x = \dot{\alpha}\,\mathbf{i} = \dot{\alpha}[1,\ 0,\ 0]^T$에 $T^0_{1x}$를 적용하면, x축 방향 벡터는 x축 회전에 의해 변하지 않으므로 그대로 $\dot{\alpha}\,\mathbf{i}$이다. 따라서:

$$\omega^{p_0}_x = -\dot{\theta}\,\mathbf{i} + \dot{\alpha}\,\mathbf{i} = (\dot{\alpha} - \dot{\theta})\,\mathbf{i} \tag{14}$$

$$\omega^{p_0}_y = -\dot{\phi}\,\mathbf{j} + \dot{\beta}\,\mathbf{j} = (\dot{\beta} - \dot{\phi})\,\mathbf{j} \tag{16}$$

**물리적 의미**: 지면 기준으로 진자의 실제 회전 속도는, 진자 자체의 회전 속도 $\dot{\alpha}$에서 구의 회전 속도 $\dot{\theta}$를 뺀 것이다. 구가 같이 돌아가고 있으므로 그 효과를 빼줘야 절대 각속도가 된다.

$R_f$에서 정의된 $\omega^p$는 $\omega^{p_0}$와 동일하다. (2.2절의 위치벡터와 같은 이유)

---

#### 진자 질량 중심의 선속도 (논문 식 17, 18)

진자 질량 중심의 선속도는 **구 중심의 속도 + 진자의 회전에 의한 속도**이다:

$$\mathbf{v}^p = \mathbf{v}^s + \omega^p \times \mathbf{r}^p$$

**x-subsystem** 각 항 대입:
- $\mathbf{v}^s_x = -R\dot{\theta}\,\mathbf{j}$
- $\omega^p_x = (\dot{\alpha}-\dot{\theta})\,\mathbf{i}$
- $\mathbf{r}^p_x = l\sin(\alpha-\theta)\,\mathbf{j} - l\cos(\alpha-\theta)\,\mathbf{k}$

외적 계산 ($\mathbf{i}\times\mathbf{j} = \mathbf{k}$, $\mathbf{i}\times\mathbf{k} = -\mathbf{j}$ 이용):

$$\omega^p_x \times \mathbf{r}^p_x = (\dot{\alpha}-\dot{\theta})\left[l\sin(\alpha-\theta)\,\mathbf{k} + l\cos(\alpha-\theta)\,\mathbf{j}\right]$$

$\mathbf{v}^s_x$와 합치면:

$$\boxed{\mathbf{v}^p_x = \left[-R\dot{\theta} + l\cos(\alpha-\theta)(\dot{\alpha}-\dot{\theta})\right]\mathbf{j} + \left[l\sin(\alpha-\theta)(\dot{\alpha}-\dot{\theta})\right]\mathbf{k}} \tag{17}$$

**y-subsystem** 각 항 대입:
- $\mathbf{v}^s_y = R\dot{\phi}\,\mathbf{i}$
- $\omega^p_y = (\dot{\beta}-\dot{\phi})\,\mathbf{j}$
- $\mathbf{r}^p_y = -l\sin(\beta-\phi)\,\mathbf{i} - l\cos(\beta-\phi)\,\mathbf{k}$

외적 계산 ($\mathbf{j}\times\mathbf{i} = -\mathbf{k}$, $\mathbf{j}\times\mathbf{k} = \mathbf{i}$ 이용):

$$\omega^p_y \times \mathbf{r}^p_y = (\dot{\beta}-\dot{\phi})\left[l\sin(\beta-\phi)\,\mathbf{k} - l\cos(\beta-\phi)\,\mathbf{i}\right]$$

$\mathbf{v}^s_y$와 합치면:

$$\boxed{\mathbf{v}^p_y = \left[R\dot{\phi} - l\cos(\beta-\phi)(\dot{\beta}-\dot{\phi})\right]\mathbf{i} + \left[l\sin(\beta-\phi)(\dot{\beta}-\dot{\phi})\right]\mathbf{k}} \tag{18}$$

---

#### 여기까지의 흐름 정리

라그랑지안을 세우려면 운동에너지 $T = \frac{1}{2}mv^2 + \frac{1}{2}I\omega^2$가 필요하고, 이를 위해 구와 진자의 속도가 모두 필요하다. 식 (5)~(18)이 그 준비 과정이다.

| 식 | 내용 |
|---|---|
| (5), (7) | 구의 각속도 |
| (6), (8) | 구 중심의 선속도 |
| (9), (10) | $R_{f_1}$ 기준 진자 위치 |
| (11), (12) | $R_{f_0}$ 기준 진자 위치 |
| (13), (15) | $R_{f_1}$ 기준 진자 각속도 |
| (14), (16) | $R_{f_0}$ 기준 진자 각속도 |
| (17), (18) | 진자 질량 중심의 선속도 |

이 속도들을 이용해 다음 단계에서 라그랑지안을 구성하고 운동방정식을 유도한다.



---

## 3. Decoupled Dynamic Equations (논문 Section 3.1.2)

### 3.1 디커플링의 핵심 가정

논문은 시스템을 단순화하기 위해 두 가지를 가정한다.

1. **횡방향과 종방향 회전 간의 동적 상호작용을 무시한다.** 즉 x-subsystem과 y-subsystem이 서로 독립적으로 동작한다고 본다.
2. **수직 z축 주위의 회전($\psi$)은 무시한다.** 횡방향·종방향 회전에 비해 negligible하다고 가정한다.

이 두 가정 덕분에 하나의 복잡한 coupled system 대신 **두 개의 독립적인 subsystem**으로 나눠서 운동방정식을 쓸 수 있게 된다.

---

### 3.2 비홀로노믹 제약식 처리 방식

일반적인 라그랑주 방법에서 미끄러짐 없는 구름(rolling without slipping)과 같은 **비홀로노믹 제약식**은 라그랑주 승수(Lagrange multiplier)로 처리해야 한다. 제약식을 라그랑지안에 미리 대입해버리면 잘못된 운동방정식이 나오기 때문이다 (논문 2.1절에서도 이 점을 명시적으로 경고하고 있음).

그런데 이 논문에서는 **처음부터 디커플링이라는 근사 모델을 채택**했기 때문에 상황이 다르다. 이미 근사 단계에서 모델 오차가 발생하므로, 그 위에서 비홀로노믹 제약식을 미리 대입해도 추가적인 오차가 발생하지 않는다. 즉:

- 완전한 정확한 모델을 다루는 경우 → 비홀로노믹 제약식을 라그랑지안에 미리 대입하면 **오류 발생**
- 디커플링 근사 모델을 다루는 경우 → 이미 근사가 적용된 상태이므로 비홀로노믹 제약식을 **미리 대입해도 무방**

이 덕분에 논문은 구르기 조건에서 유도한 $\dot{x} = -R\dot{\theta}$, $\dot{y} = R\dot{\phi}$ 관계를 라그랑지안 구성에 직접 사용할 수 있다.

---

### 3.3 라그랑지안 구성

라그랑지안은 다음과 같이 정의된다:

$$L = E_k - E_p \tag{19}$$

#### 관성 모멘트

**구 (hollow spherical shell)**:

$$I^s = \frac{2}{3}M_s R^2 \tag{21a}$$

속이 꽉 찬 구(solid sphere)는 $\frac{2}{5}MR^2$이지만, 이 로봇의 구는 속이 빈 껍데기이므로 질량이 표면에 집중되어 계수가 더 크다.

| 형태 | 관성 모멘트 |
|---|---|
| Solid sphere | $\frac{2}{5}MR^2$ |
| Hollow spherical shell | $\frac{2}{3}MR^2$ |

**진자 (slender rod, 회전축: 구 중심)**:

$$I^p = \frac{1}{12}m_p l^2 + m_p\left(\frac{l}{2}\right)^2 = \frac{1}{3}m_p l^2 \tag{21b}$$

$\frac{1}{12}m_p l^2$는 질량 중심 기준 관성 모멘트이고, $m_p(l/2)^2$는 **평행축 정리(parallel axis theorem)** 적용 항이다. 진자의 회전축이 구 중심이고 진자 질량 중심까지의 거리가 $l/2$이므로:

$$I = I_{cm} + md^2 \implies I^p = \frac{1}{12}m_p l^2 + m_p\left(\frac{l}{2}\right)^2 = \frac{1}{3}m_p l^2$$

이는 막대의 한쪽 끝을 기준으로 회전할 때의 공식 $\frac{1}{3}ml^2$과 동일하다. 구 중심이 진자의 한쪽 끝에 해당하므로 당연한 결과다.

#### 전체 운동에너지

$$E_k = \frac{1}{2}M_s\|\mathbf{v}^s\|^2 + \frac{1}{2}I^s\|\boldsymbol{\omega}^s\|^2 + \frac{1}{2}m_p\|\mathbf{v}^p\|^2 + \frac{1}{2}I^p\|\boldsymbol{\omega}^p\|^2 \tag{20}$$

#### 전체 퍼텐셜에너지

$$E_p = m_p g\, r^p_z \tag{22}$$

$r^p_z$는 진자 질량 중심의 수직 위치이다. 기준점이 구 중심이므로 구 자체의 퍼텐셜 에너지는 0이다.

---

### 3.4 디커플링된 라그랑지안

앞서 구한 속도 표현들(식 6, 8, 17, 18)을 $E_k$, $E_p$에 대입하면 라그랑지안이 x-subsystem과 y-subsystem으로 완전히 분리된다.

**x-subsystem** (횡방향, $\theta$와 $\alpha$):

$$L_x = \frac{1}{2}M_s(-R\dot{\theta})^2 + \frac{1}{2}I^s(-\dot{\theta})^2$$

$$+ \frac{1}{2}m_p\left[(-R\dot{\theta} + l\cos(\alpha-\theta)(\dot{\alpha}-\dot{\theta}))^2 + (l\sin(\alpha-\theta)(\dot{\alpha}-\dot{\theta}))^2\right]$$

$$+ \frac{1}{2}I^p(\dot{\alpha}-\dot{\theta})^2 + m_p g l\cos(\alpha-\theta) \tag{23}$$

**y-subsystem** (종방향, $\phi$와 $\beta$):

$$L_y = \frac{1}{2}M_s(R\dot{\phi})^2 + \frac{1}{2}I^s(-\dot{\phi})^2$$

$$+ \frac{1}{2}m_p\left[(R\dot{\phi} - l\cos(\beta-\phi)(\dot{\beta}-\dot{\phi}))^2 + (l\sin(\beta-\phi)(\dot{\beta}-\dot{\phi}))^2\right]$$

$$+ \frac{1}{2}I^p(\dot{\beta}-\dot{\phi})^2 + m_p g l\cos(\beta-\phi) \tag{24}$$

---

### 3.5 오일러-라그랑주 방정식 적용

**x-subsystem** (일반좌표: $q_1 = \theta$, $q_2 = \alpha$):

$$\frac{d}{dt}\left(\frac{\partial L_x}{\partial \dot{q}_i}\right) - \frac{\partial L_x}{\partial q_i} = Q_i \tag{25}$$

입력 토크: 진자를 회전시키는 토크 $\tau_x$는 반작용으로 구에도 같은 크기로 작용한다.

$$Q_\theta = \tau_x, \qquad Q_\alpha = \tau_x \tag{26}$$

**y-subsystem** (일반좌표: $q_1 = \phi$, $q_2 = \beta$):

$$\frac{d}{dt}\left(\frac{\partial L_y}{\partial \dot{q}_i}\right) - \frac{\partial L_y}{\partial q_i} = Q_i \tag{27}$$

$$Q_\phi = \tau_y, \qquad Q_\beta = \tau_y \tag{28}$$

---

### 3.6 행렬 형태의 운동방정식

일반좌표 벡터를 $\mathbf{q} = [\theta\ \ \alpha\ \ \phi\ \ \beta]^T$로 정의하면, 두 subsystem의 운동방정식이 다음의 행렬 형태로 합쳐진다:

$$M(\mathbf{q})\ddot{\mathbf{q}} + V(\mathbf{q}, \dot{\mathbf{q}}) = \mathbf{u} \tag{30}$$

$$\begin{bmatrix}M_{11}&M_{12}&0&0\\M_{21}&M_{22}&0&0\\0&0&M_{33}&M_{34}\\0&0&M_{43}&M_{44}\end{bmatrix} \begin{bmatrix}\ddot{q}_1\\\ddot{q}_2\\\ddot{q}_3\\\ddot{q}_4\end{bmatrix} + \begin{bmatrix}V_{11}\\V_{21}\\V_{31}\\V_{41}\end{bmatrix} = \begin{bmatrix}\tau_x\\\tau_x\\\tau_y\\\tau_y\end{bmatrix} \tag{31}$$

**M 행렬 성분** ($q_1=\theta,\ q_2=\alpha,\ q_3=\phi,\ q_4=\beta$):

$$M_{11} = M_s R^2 + m_p R^2 + m_p l^2 + I^s + I^p + 2m_p Rl\cos(q_2-q_1)$$

$$M_{12} = M_{21} = -m_p l^2 - I^p - m_p Rl\cos(q_2-q_1)$$

$$M_{22} = m_p l^2 + I^p$$

$$M_{33} = M_s R^2 + m_p R^2 + m_p l^2 + I^s + I^p + 2m_p Rl\cos(q_4-q_3)$$

$$M_{34} = M_{43} = -m_p l^2 - I^p - m_p Rl\cos(q_4-q_3)$$

$$M_{44} = m_p l^2 + I^p$$

나머지 성분은 모두 0 (디커플링의 결과)

**V 벡터 성분**:

$$V_{11} = m_p Rl\sin(q_2-q_1)\dot{q}_1^2 + m_p Rl\sin(q_2-q_1)\dot{q}_2^2 - 2m_p Rl\sin(q_2-q_1)\dot{q}_1\dot{q}_2 - m_p gl\sin(q_2-q_1)$$

$$V_{21} = m_p gl\sin(q_2-q_1)$$

$$V_{31} = m_p Rl\sin(q_4-q_3)\dot{q}_3^2 + m_p Rl\sin(q_4-q_3)\dot{q}_4^2 - 2m_p Rl\sin(q_4-q_3)\dot{q}_3\dot{q}_4 - m_p gl\sin(q_4-q_3)$$

$$V_{41} = m_p gl\sin(q_4-q_3) \tag{32}$$

---

#### M 행렬의 블록 구조 의미

M 행렬의 좌상단 $2\times2$ 블록이 x-subsystem, 우하단 $2\times2$ 블록이 y-subsystem이고 나머지는 전부 0이다. 이것이 디커플링의 수학적 표현이며, x-subsystem의 운동이 y-subsystem에 영향을 주지 않음을 의미한다.

각 성분의 물리적 의미:

- $M_{11}$, $M_{33}$: 구의 회전에 대한 유효 관성. 구 자체의 관성 + 진자의 관성 + 진자-구 결합항 포함.
- $M_{22}$, $M_{44}$: 진자의 회전에 대한 유효 관성. 진자 자체의 관성만.
- $M_{12} = M_{21}$: 구와 진자 사이의 결합항. $\cos(q_2-q_1)$ 항이 포함되어 비선형이다.

---

#### V 벡터 성분의 물리적 배경: 회전 좌표계에서의 가속도 분해

V 벡터의 각 항이 어디서 비롯되는지 이해하려면 **회전 좌표계에서의 가속도 분해**를 알아야 한다.

##### Transport theorem (복습)

관성계에서 벡터 $\mathbf{A}$의 시간 미분:

$$\left(\frac{d\mathbf{A}}{dt}\right)_{\text{inertial}} = \left(\frac{d\mathbf{A}}{dt}\right)_{\text{body}} + \boldsymbol{\omega} \times \mathbf{A}$$

이를 위치벡터 $\mathbf{r}$에 두 번 적용하면 관성계 기준 가속도가 나온다.

**1회 적용 (속도)**:

$$\mathbf{v}_{\text{inertial}} = \dot{\mathbf{r}}_{\text{body}} + \boldsymbol{\omega} \times \mathbf{r}$$

**2회 적용 (가속도)**:

$$\mathbf{a}_{\text{inertial}} = \ddot{\mathbf{r}}_{\text{body}} + 2\boldsymbol{\omega} \times \dot{\mathbf{r}}_{\text{body}} + \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r}) + \dot{\boldsymbol{\omega}} \times \mathbf{r}$$

##### 각 항의 물리적 의미

**$\ddot{\mathbf{r}}_{\text{body}}$ : 상대 가속도**

회전 좌표계 안에서 관찰한 순수한 가속도. 좌표계가 회전하지 않는다고 가정했을 때의 가속도이다.

**$\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r})$ : 구심 가속도 → 원심력**

크기는 $\omega^2 r$이고 방향은 회전축을 향하는 안쪽이다. 원운동을 유지하려면 반드시 필요한 가속도이며, 이에 대응하는 힘이 구심력이다.

회전 좌표계 안에서 관찰하는 사람 입장에서는 이 구심가속도가 마치 **바깥으로 밀리는 힘**처럼 느껴지는데, 그것이 바로 **원심력**이다. 실제로 존재하는 힘이 아니라 회전 좌표계에서만 나타나는 가상의 힘이다.

**$2\boldsymbol{\omega} \times \dot{\mathbf{r}}_{\text{body}}$ : 코리올리 가속도 → 코리올리 힘**

회전 좌표계 안에서 물체가 움직일 때 추가로 나타나는 가속도이다. $\dot{\mathbf{r}}_{\text{body}} = 0$이면 사라지므로, 좌표계 안에서 물체가 움직이고 있을 때만 나타난다.

방향은 속도와 회전축 모두에 수직이다. 계수 2가 붙는 이유는 좌표계 회전 효과가 속도를 미분할 때 한 번, 위치를 미분할 때 한 번, 총 두 번 중첩되기 때문이다.

**$\dot{\boldsymbol{\omega}} \times \mathbf{r}$ : 오일러 가속도**

각속도 자체가 변할 때 나타나는 항이다. 등속 회전이면 $\dot{\boldsymbol{\omega}} = 0$이므로 사라진다.

##### V 벡터 성분과의 대응

V 벡터의 각 항이 위 가속도 성분들로부터 어떻게 나오는지 $V_{11}$을 예로 보면:

$$V_{11} = m_p Rl\sin(q_2-q_1)\dot{q}_1^2 + m_p Rl\sin(q_2-q_1)\dot{q}_2^2 - 2m_p Rl\sin(q_2-q_1)\dot{q}_1\dot{q}_2 - m_p gl\sin(q_2-q_1)$$

| 항 | 물리적 의미 |
|---|---|
| $\dot{q}_1^2$ 항 | 구의 회전 $\theta$에 의한 **원심력** 효과 |
| $\dot{q}_2^2$ 항 | 진자 자체 회전 $\alpha$에 의한 **원심력** 효과 |
| $\dot{q}_1\dot{q}_2$ 항 | 두 회전의 상호작용에 의한 **코리올리 힘** (계수 2가 특징) |
| $m_p gl\sin(q_2-q_1)$ 항 | **중력**에 의한 복원력 |

라그랑지안을 편미분하면 이 항들이 자동으로 나온다. 뉴턴 역학으로 직접 힘을 분해해도 같은 결과가 나오지만, 라그랑지안 방법이 훨씬 체계적으로 모든 항을 챙겨준다.

V 벡터는 단순히 "비선형 항들의 모음"이 아니라 **회전 좌표계에서 반드시 나타나는 가상 힘들의 집합**이다. 뒤에 나오는 피드백 선형화에서 $V$를 제어 입력으로 상쇄한다는 것은, 이 원심력과 코리올리 힘을 모터 토크로 정확히 맞받아친다는 물리적 의미를 갖는다.


---





## 4. Radius of Curvature over Curvilinear Trajectories (논문 Section 3.2)

### 4.1 개요

구형 로봇이 곡선 궤적을 따라 이동할 때, 곡률 반경(radius of curvature) $e$는 주행 속도와 진자의 조향각에 의해 결정된다. 이 절에서는 구에 작용하는 힘들의 균형으로부터 곡률 반경을 해석적으로 구한다.

---

### 4.2 곡선 주행 중의 각속도 (식 33)

곡선 궤적을 따라 이동하는 구가 수직 z축 주위로 회전할 때의 각속도 $\Omega$는:

$$\Omega = \frac{-R\dot{\theta}}{e} \tag{33}$$

**유도**: 원운동 선속도 공식 $v = e\Omega$와 구르기 조건 $v = R\dot{\theta}$를 결합하면:

$$R\dot{\theta} = e\Omega \implies \Omega = \frac{R\dot{\theta}}{e}$$

음수 부호는 앞서 정의한 부호 convention에 따른 것이다.
![[Pasted image 20260403032325.png]]

---

### 4.3 힘의 균형 (식 34)

구와 지면 사이에 작용하는 측면 마찰력 $F_f$는 구와 진자에 작용하는 원심력의 합과 같다.

**원심력 = 구심력 배경 개념**

곡선 운동에서 가속도를 접선(tangential)과 법선(normal) 성분으로 분해하면, 법선 방향 가속도(구심가속도)의 크기는:

$$a_n = \frac{\dot{s}^2}{r} = \frac{v^2}{r} = r\omega^2$$

따라서 구심력(= 원심력의 크기) $= mr\omega^2$.

**힘 균형**:

$$F_f = F_{c1} + F_{c2} = M_s \cdot e \cdot \Omega^2 + m_p(e - l\sin(\beta-\phi))\Omega^2 \tag{34}$$

여기서 진자의 이동량 $l\sin(\beta-\phi)$이 곡률 반경 $e$에 비해 훨씬 작다고 가정하면:

$$l\sin(\beta-\phi) \ll e \implies l\sin(\beta-\phi) \text{ 항 무시}$$

$$F_f \approx (M_s + m_p) \cdot e \cdot \Omega^2$$

---

### 4.4 토크 균형으로부터 곡률 반경 유도

#### 토크 $T_1$: 종방향 축 주위 (식 35)

조향(steering)은 $\beta$ (y축 주위 진자 회전)에 의해 일어난다. 진자가 y축 주위로 기울면 구가 좌우로 쏠리면서 곡선 주행이 발생한다.

이때 진자 무게, 원심력, 마찰력이 모두 $\mathbf{i}$-$\mathbf{k}$ 평면 안에서 작용하므로, 이 힘들이 만드는 토크의 회전축은 **종방향 축($\mathbf{j}$, $\mathbf{J}$)** 이다. 오른손 법칙에 의해 평면 안의 힘은 그 평면에 수직인 축 주위 토크를 만들기 때문이다. (즉 논문의 Torque acting around the transversal axis of the sphere 이 표현이 잘못됨.)

구의 종방향 축 주위로 작용하는 토크:

$$T_1 \approx m_p gl\sin(\beta-\phi) + m_p e\Omega^2 l\cos(\beta-\phi) - R(M_s+m_p)e\Omega^2 \tag{35}$$

각 항의 의미:
- $m_p gl\sin(\beta-\phi)$ : 진자 무게에 의한 토크
- $m_p e\Omega^2 l\cos(\beta-\phi)$ : 진자 원심력에 의한 토크
- $-R(M_s+m_p)e\Omega^2$ : 마찰력에 의한 반작용 토크

#### 토크 $T_2$: 각운동량의 시간 미분 (식 36-38)

구의 각속도를 지면 고정 좌표계의 단위벡터 $\mathbf{I}, \mathbf{J}, \mathbf{K}$로 표현하면:

$$\boldsymbol{\omega} = \Omega\mathbf{K} - \dot{\phi}\mathbf{J} - \dot{\theta}\mathbf{i} = -\dot{\theta}\cos\phi\,\mathbf{I} - \dot{\phi}\,\mathbf{J} + (\Omega - \dot{\theta}\sin\phi)\mathbf{K} \tag{36}$$

각운동량:

$$\mathbf{L} = I^s\boldsymbol{\omega} = -I^s\dot{\theta}\cos\phi\,\mathbf{I} - I^s\dot{\phi}\,\mathbf{J} + I^s(\Omega - \dot{\theta}\sin\phi)\mathbf{K} \tag{37}$$

토크는 각운동량의 시간 미분인데, 여기서 **Transport theorem**을 적용한다.

**Transport theorem 배경 개념**

관성계에서 벡터 $\mathbf{A}$의 시간 미분은 body 프레임 기준 미분과 회전 효과의 합이다:

$$\left(\frac{d\mathbf{A}}{dt}\right)_{\text{inertial}} = \left(\frac{d\mathbf{A}}{dt}\right)_{\text{body}} + \boldsymbol{\omega} \times \mathbf{A}$$

이는 회전좌표계의 기저벡터 $\mathbf{e}_1, \mathbf{e}_2, \mathbf{e}_3$가 시간에 따라 변하기 때문이다. 벡터 $\mathbf{A} = A_1\mathbf{e}_1 + A_2\mathbf{e}_2 + A_3\mathbf{e}_3$를 관성계에서 미분하면:

$$\left(\frac{d\mathbf{A}}{dt}\right)_{\text{inertial}} = \underbrace{(\dot{A}_1\mathbf{e}_1 + \dot{A}_2\mathbf{e}_2 + \dot{A}_3\mathbf{e}_3)}_{\left(\frac{d\mathbf{A}}{dt}\right)_{\text{body}}} + \underbrace{A_1\dot{\mathbf{e}}_1 + A_2\dot{\mathbf{e}}_2 + A_3\dot{\mathbf{e}}_3}_{= \boldsymbol{\omega} \times \mathbf{A}}$$

논문에서는 구가 **등속 원운동**을 한다고 가정하므로 $\left(\frac{d\mathbf{L}}{dt}\right)_{\text{body}} = 0$. 따라서:

$$T_2 = \frac{d\mathbf{L}}{dt}\bigg|_{\text{inertial}} = \boldsymbol{\Omega} \times \mathbf{L} = I^s\Omega\dot{\phi}\,\mathbf{I} - I^s\Omega\dot{\theta}\cos\phi\,\mathbf{J} \tag{38}$$

$T_2$의 $\mathbf{J}$ 방향 성분이 $-I^s\Omega\dot{\theta}\cos\phi$로 나오는데, 이것이 $T_1$과 균형을 이루는 항이다.

#### 곡률 반경 $e$ 도출 (식 39)

$T_1$과 $T_2$의 $\mathbf{J}$ 방향 성분을 같다고 놓고, $\phi$가 작다고 가정($\cos\phi \approx 1$, $\sin\phi \approx 0$)하면:

$$-I^s\Omega\dot{\theta} \approx m_p gl\sin\beta + m_p e\Omega^2 l\cos\beta - R(M_s+m_p)e\Omega^2$$

식 (33)의 $\Omega = -R\dot{\theta}/e$를 대입하여 정리하면:

$$\boxed{e \approx \frac{R\dot{\theta}^2\left[I^s - m_p Rl\cos\beta\right] + R\dot{\theta}^2\left[R^2(M_s+m_p)\right]}{m_p gl\sin\beta}} \tag{39}$$

---
### 4.5 물리적 의미

식 (39)에서 곡률 반경 $e$는 두 가지 변수에 의해 결정된다.

- **$\dot{\theta}$** (구동 각속도): $\dot{\theta}^2$에 비례하므로 속도가 빠를수록 곡률 반경이 커진다. 즉 빠르게 달릴수록 덜 꺾인다.
- **$\beta$** (진자 조향각): $\sin\beta$가 분모에 있으므로 $\beta$가 클수록 곡률 반경이 작아진다. 즉 진자를 많이 기울일수록 더 급하게 꺾인다.

이 관계를 활용하면 원하는 곡률 반경에 맞게 진자 각도와 구동 속도를 설정할 수 있어, 이후 궤적 생성(trajectory generation)의 기초가 된다.

![[Pasted image 20260403190951.png|506]]



---

### 4.6 원형 궤적 시뮬레이션 구현 시의 근사 문제

#### 잘못된 근사: $\phi_d = \Omega t$

초기 구현에서는 y-subsystem의 목표값으로 $\phi_d = \Omega t$를 사용했다. 이는 논문의 주장이 아니라 구현 과정에서 임의로 도입된 근사이며, **구르기 조건을 무시**한 것이다.

#### 구르기 조건으로부터 올바른 $\phi_d$ 유도

논문 2.1절에서 유도한 y-subsystem 구르기 조건:

$$\mathbf{v}^s_y = R\dot{\phi}\,\mathbf{i} \tag{구르기 조건}$$

$\phi$는 y축 기준 회전각이므로, y축 기준 구름에 의해 생기는 속도는 **$\mathbf{i}$ (x) 방향**이다. 즉 $\dot\phi$가 만들어내는 선속도는 x(i)방향 속도이다.

원운동에서 구가 $\mathbf{j}$ 방향을 향해 출발할 때, 시간 $t$에서 구의 x(i)방향 속도는:

$$v_{\mathbf{i}} = v_d \sin(\Omega t) \tag{원운동 기하학}$$

두 식을 연결하면 목표 $\dot{\phi}$가 결정된다:

$$\dot{\phi}_d = \frac{v_d}{R}\sin(\Omega t)$$

초기 조건 $\phi_d(0) = 0$으로 적분하면:

$$\phi_d(t) = -\frac{v_d}{R\Omega}\cos(\Omega t) + \frac{v_d}{R\Omega} = \frac{v_d}{R\Omega}\bigl(1 - \cos(\Omega t)\bigr)$$

$e = v_d/|\Omega|$ (곡률 반경)을 이용하면, $\Omega < 0$일 때 $v_d/(R\Omega) = -e/R$이므로:

$$\boxed{\phi_d(t) = -\frac{e}{R}\bigl(1 - \cos(\Omega t)\bigr)}$$

$\ddot{\phi}_d$는:

$$\ddot{\phi}_d = \frac{v_d}{R}\,\Omega\cos(\Omega t)$$

#### $R_{f_0}$의 제약과 $\phi$, $\psi$의 관계

$R_{f_0}$는 회전이 불가한 기준 프레임(inertially fixed frame)이다. $\phi$는 이 프레임에서 정의된 y축 기준 구름각이고, $\psi$는 구의 heading angle(수직축 기준 방향각)이다. 이 둘은 구르기 조건 $v_{\mathbf{i}} = R\dot\phi$를 통해 간접적으로 연결될 뿐, **직접 동일시할 수 없다.** 초기 근사 $\phi_d = \Omega t$는 $\phi \equiv \psi$로 놓은 것으로, 구르기 조건에서 나오는 $R$ 스케일링 인자를 무시한 것이다.

---




## 5. Control of Spherical Rolling Motion (논문 Section 4)

### 5.1 Feedback Linearization (피드백 선형화)

#### 기본 아이디어

피드백 선형화(feedback linearization)는 비선형 시스템을 적절한 피드백을 통해 **대수적으로 선형 시스템으로 변환**하는 폐루프 제어 설계 방법이다. 변환된 선형 시스템은 이후 선형 제어 기법으로 쉽게 제어할 수 있다. 로보틱스에서는 **computed torque control**이라는 이름으로 널리 알려져 있다.

---

#### 운동방정식과 제어 입력 설계

기계 시스템의 운동방정식:

$$M(\mathbf{q})\ddot{\mathbf{q}} + V(\mathbf{q}, \dot{\mathbf{q}}) = \mathbf{u} \tag{40}$$

$\mathbf{u}$는 우리가 자유롭게 설계할 수 있는 제어 입력(토크)이다. $M$과 $V$ 모두 비선형 항이다. $M$ 안에는 $\cos(q_2 - q_1)$ 같은 비선형 항이 포함되어 있고, $V$는 원심력, 코리올리 힘, 중력 등 속도와 위치에 의존하는 비선형 항들로 구성된다.

목표는 $\mathbf{q}(t)$가 원하는 궤적 $\mathbf{q}_d(t)$를 따라가도록 $\mathbf{u}$를 설계하는 것이다.

다음의 **linearizing control law**를 설계한다:

$$\mathbf{u}(t) = V(\mathbf{q}, \dot{\mathbf{q}}) + K_v\dot{\mathbf{e}}(t) + K_p\mathbf{e}(t) + M(\mathbf{q})\ddot{\mathbf{q}}_d(t) \tag{41}$$

여기서 $\mathbf{e}(t) = \mathbf{q}_d(t) - \mathbf{q}(t)$는 추종 오차이고, $K_p$와 $K_v$는 양정치(positive definite) 피드백 이득이다.

- $K_p$ (proportional gain): 위치 오차에 곱해지는 이득. 오차가 크면 더 세게 밀어붙이는 역할.
- $K_v$ (velocity gain): 속도 오차에 곱해지는 이득. 오차가 빠르게 변할 때 제동을 거는 역할. PD 제어기의 P, D 이득과 같은 개념이다.

![[Pasted image 20260403210639.png|532]]


---

#### 비선형 항 상쇄 과정

식 (41)을 식 (40)에 대입하면:

$$M\ddot{\mathbf{q}} + \cancel{V} = \cancel{V} + M\ddot{\mathbf{q}}_d + K_v\dot{\mathbf{e}} + K_p\mathbf{e}$$

$$M(\ddot{\mathbf{q}} - \ddot{\mathbf{q}}_d) = K_v\dot{\mathbf{e}} + K_p\mathbf{e}$$

$\ddot{\mathbf{e}} = \ddot{\mathbf{q}}_d - \ddot{\mathbf{q}}$이므로:

$$-M\ddot{\mathbf{e}} = K_v\dot{\mathbf{e}} + K_p\mathbf{e}$$

양변에 $M^{-1}$을 곱하면:

$$-\ddot{\mathbf{e}} = M^{-1}K_v\dot{\mathbf{e}} + M^{-1}K_p\mathbf{e}$$

---

#### $M$의 비선형성은 어떻게 처리되는가

$M^{-1}$이 앞에 붙으면 $K_p$, $K_v$가 비선형 $M^{-1}$과 뒤섞여서 깔끔한 선형 방정식이 안 나오는 것처럼 보인다. 이를 해결하는 방법은 $K_p$와 $K_v$를 스칼라가 아니라 **$M$을 포함한 행렬로 설계**하는 것이다:

$$K_p \to M(\mathbf{q})\tilde{K}_p, \qquad K_v \to M(\mathbf{q})\tilde{K}_v$$

이렇게 놓으면:

$$-\ddot{\mathbf{e}} = M^{-1} \cdot M\tilde{K}_v\dot{\mathbf{e}} + M^{-1} \cdot M\tilde{K}_p\mathbf{e} = \tilde{K}_v\dot{\mathbf{e}} + \tilde{K}_p\mathbf{e}$$

$M^{-1}M = I$로 상쇄되어 완전히 선형화된다.

논문에서 $K_p$, $K_v$를 스칼라처럼 단순하게 표기한 데는 두 가지 이유가 있다.

첫째, **디커플링 구조** 덕분이다. $M$ 행렬이 블록 대각 구조라 각 subsystem이 독립적으로 작동하고, 각 $2\times2$ 블록 안에서 $M^{-1}K_p$ 처리가 분리되어 단순해진다.

둘째, **실용적 근사**이다. $K_p$, $K_v$를 충분히 크게 잡으면 $M$의 비선형성 영향이 상대적으로 작아져서 근사적으로 선형처럼 동작한다. 논문이 이를 묵시적으로 가정하고 있다.

최종 오차 동역학:

$$\ddot{\mathbf{e}} + K_v\dot{\mathbf{e}} + K_p\mathbf{e} = 0 \tag{42}$$

---

#### 오차 동역학의 의미: 스프링-댐퍼 시스템

식 (42)는 **스프링-댐퍼 시스템**과 동일한 형태이다.

스프링-댐퍼 시스템의 운동방정식:

$$m\ddot{x} + c\dot{x} + kx = 0$$

각 항의 물리적 의미:
- $c\dot{x}$: 댐퍼가 속도에 비례하여 저항하는 힘
- $kx$: 스프링이 변위에 비례하여 복원하는 힘

$m = 1$, $c = K_v$, $k = K_p$로 놓으면 오차 동역학과 완전히 같은 형태가 된다.

- $K_p\mathbf{e}$: 오차가 크면 강하게 복원 (스프링 역할)
- $K_v\dot{\mathbf{e}}$: 오차가 빠르게 변하면 제동 (댐퍼 역할)

스프링-댐퍼 시스템이 초기 변위에서 반드시 평형점($x=0$)으로 수렴하듯, $K_p$와 $K_v$를 양수로 잡으면 오차 $\mathbf{e}(t) \to 0$으로 수렴이 수학적으로 보장된다.

$K_p = \omega_n^2$, $K_v = 2\zeta\omega_n$으로 설정하면 원하는 자연주파수($\omega_n$)와 감쇠비($\zeta$)를 갖는 응답을 자유롭게 지정할 수 있다.

---

#### 제어 입력의 물리적 의미

$$\mathbf{u} = \underbrace{V(\mathbf{q}, \dot{\mathbf{q}}) + M(\mathbf{q})\ddot{\mathbf{q}}_d}_{\text{비선형 보상}} + \underbrace{K_v\dot{\mathbf{e}} + K_p\mathbf{e}}_{\text{오차 보정}}$$

제어 입력은 두 파트로 구성된다.

**비선형 보상 파트** $V + M\ddot{\mathbf{q}}_d$: 현재 구와 진자의 상태($\mathbf{q}$, $\dot{\mathbf{q}}$)를 측정하여 지금 이 순간 시스템의 비선형 동역학을 실시간으로 계산하고 그것을 토크로 상쇄한다. 물리적으로는 "중력, 코리올리 힘, 원심력 등이 지금 이렇게 작용하고 있으니 모터 토크로 정확히 맞받아쳐서 없애버린다"는 의미이다.

**오차 보정 파트** $K_v\dot{\mathbf{e}} + K_p\mathbf{e}$: 목표 궤적과 현재 상태의 차이를 보고 추가 토크를 더한다. 뒤처지면 더 세게 밀고, 너무 빠르면 제동을 건다.

결국 모터는 매 순간 두 가지 일을 동시에 수행한다. 시스템의 복잡한 비선형 물리를 실시간으로 상쇄하면서, 동시에 목표 궤적과의 오차를 줄이는 토크를 출력하는 것이다.

---

### 5.2 Fuzzy Control (퍼지 제어)

#### 퍼지 제어를 추가하는 이유

실제 물리 시스템에서는 모델링 오차, 파라미터 불확실성, 외란 등이 존재한다. 피드백 선형화는 정확한 모델을 가정하므로, 모델 오차가 있을 경우 비선형 항의 상쇄가 완전하지 않아 성능이 저하될 수 있다. 퍼지 제어를 추가하면 이러한 불확실성에 대한 **강인성(robustness)** 을 높일 수 있다.

---

#### 퍼지 제어기 구조

논문에서 사용하는 퍼지 제어기는 **2입력 1출력** 구조이다.

- 입력 1: 위치 오차 $e$
- 입력 2: 속도 오차 $\dot{e}$ (오차의 변화율)
- 출력: 이득 조정값

퍼지 제어기의 역할은 고정된 PD 이득 $K_p$, $K_v$를 **시변(time-varying) 파라미터**로 바꾸는 것이다. 즉 오차의 크기와 방향에 따라 $K_p$, $K_v$를 실시간으로 조정한다.

![[Pasted image 20260403210716.png|507]]

---

#### 멤버십 함수와 룰 베이스

입력과 출력의 멤버십 함수는 **삼각형 함수(triangular function)** 로 정의된다. 언어 변수는 NL, NM, NS, ZR, PS, PM, PL (Negative Large ~ Positive Large) 7개로 구성된다.

룰 베이스 (Rule base):

| $e$ \\ $\dot{e}$ | NL | NM | NS | ZR | PS | PM | PL |
|---|---|---|---|---|---|---|---|
| PL | ZR | PS | PM | PL | PL | PL | PL |
| PM | NS | ZR | PS | PM | PL | PL | PL |
| PS | NM | NS | ZR | PS | PM | PL | PL |
| ZR | NL | NM | NS | ZR | PS | PM | PL |
| NS | NL | NL | NM | NS | ZR | PS | PM |
| NM | NL | NL | NL | NM | NS | ZR | PS |
| NL | NL | NL | NL | NL | NM | NS | ZR |

![[Pasted image 20260403210132.png|569]]


룰의 해석 예시: 오차가 PL(크게 양수)이고 오차 변화율이 ZR(0)이면 출력은 PL(크게 양수) → 이득을 크게 키워서 빠르게 보정.

---

#### 전체 제어 구조 요약

$$\mathbf{u}(t) = V(\mathbf{q}, \dot{\mathbf{q}}) + K_v(e, \dot{e})\dot{\mathbf{e}}(t) + K_p(e, \dot{e})\mathbf{e}(t) + M(\mathbf{q})\ddot{\mathbf{q}}_d(t)$$

기본 피드백 선형화 구조(식 41)에서 고정 이득 $K_p$, $K_v$가 퍼지 제어기에 의해 실시간으로 결정되는 시변 이득 $K_p(e, \dot{e})$, $K_v(e, \dot{e})$로 대체된다.

---





## 6. Simulation Results (논문 Section 5)

### 6.1 시뮬레이션 파라미터

논문에서 사용한 시스템 수치 파라미터:

| 파라미터 | 값 |
|---|---|
| $M_s$ (구의 질량) | 3 kg |
| $m_p$ (진자의 질량) | 2 kg |
| $R$ (구의 반지름) | 0.2 m |
| $l$ (진자 길이) | 0.075 m |
| $g$ | 9.81 m/s² |
| 최대 입력 토크 | 2.5 Nm (saturation) |

샘플링 주기:
- 직선 궤적: 0.001 s
- 곡선 궤적: 0.001 s, 0.1 s, 0.15 s (샘플링 주기 영향 분석용)

---

### 6.2 직선 궤적 추종 (Rolling over Linear Trajectories)

#### PD 이득 선택 과정

먼저 $K_v = 0.8$로 고정하고 $K_p$를 변화시키면서 step response를 비교한다.

- $K_p$가 클수록 응답이 빨라지지만 오버슈트(overshoot)도 커진다.
- 반대로 $K_p$가 작으면 오버슈트는 줄지만 응답이 느려진다.

이득 선택 기준: **입력 토크가 actuator 최대값 2.5 Nm을 넘지 않는 범위** 에서 가장 빠른 응답을 갖도록 설정. 최종적으로 $K_p = 1$, $K_v = 1$로 결정.

#### PD vs PD-Fuzzy 비교

동일한 $K_p = 1$, $K_v = 1$ 기준으로 일반 PD 제어기와 PD-Fuzzy 제어기(PDFC)를 비교한 결과:

- **상승 시간(rise time)**: PDFC가 더 짧다.
- **오버슈트(overshoot)**: PDFC가 더 작다.
- **정착 시간(settling time)**: PDFC가 더 짧다.
- **필요 토크**: PDFC가 전체적으로 더 작은 토크를 요구한다.
- 퍼지 제어기를 사용하면 속도 오차가 7초 이내에 수렴한다.

오버슈트가 여전히 약 20% 수준으로 남아 있는 점은 논문이 한계로 인정하고 있으며, gray prediction 같은 추가 제어 기법으로 보완할 수 있다고 언급한다.

---

### 6.3 곡선 궤적 추종 (Rolling over Curvilinear Trajectories)

원형 궤적(circular trajectory) 추종 시뮬레이션을 PD와 PDFC로 각각 수행하고, 샘플링 주기별로 비교한다.

#### 샘플링 주기의 영향

| 샘플링 주기 | 추종 오차 |
|---|---|
| 0.001 s | 매우 작음 |
| 0.1 s | 증가 |
| 0.15 s | 더 증가 |

샘플링 주기가 커질수록 추종 오차가 증가한다. 이는 이산화(discretization) 오차가 누적되기 때문이다.

#### PD vs PDFC 비교

같은 샘플링 주기 조건에서 PDFC가 PD보다 항상 더 작은 추종 오차를 보인다. 퍼지 제어기의 적응적 이득 조정이 곡선 구간에서 특히 효과적으로 작용한다.

---

### 6.4 결과 요약

| 항목 | PD | PD-Fuzzy |
|---|---|---|
| 상승 시간 | 느림 | 빠름 |
| 오버슈트 | 큼 | 작음 |
| 정착 시간 | 느림 | 빠름 |
| 필요 토크 | 상대적으로 큼 | 상대적으로 작음 |
| 곡선 추종 오차 | 큼 | 작음 |

디커플링 동역학 모델과 피드백 선형화 기반 퍼지 PD 제어기의 조합이 직선 및 곡선 궤적 모두에서 유효하게 작동함을 시뮬레이션으로 검증하였다.