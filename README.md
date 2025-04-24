<h1>get_calibrate_camera</h1>

<p>This is a ROS 2 package designed for camera calibration and streaming functionality.</p>

<h2>Features</h2>
<ul>
    <li>USB camera node (<code>usbcam_node</code>)</li>
    <li>IP camera streaming node (<code>ip_stream_node</code>)</li>
    <li>Camera calibration functionality (<code>calibrate_camera</code>)</li>
</ul>

<h2>Installation</h2>
<ol>
    <li>Clone the repository:
        <pre><code>git clone git@github.com:oscarpoudel/get_calibrate_camera.git</code></pre>
    </li>
    <li>Navigate to the package directory:
        <pre><code>cd get_calibrate_camera</code></pre>
    </li>
    <li>Install dependencies:
        <pre><code>pip install -r requirements.txt</code></pre>
    </li>
</ol>

<h2>Usage</h2>
<h3>Run USB Camera Node</h3>
<pre><code>ros2 run get_calibrate_camera usbcam_node</code></pre>

<h3>Run IP Stream Node</h3>
<pre><code>ros2 run get_calibrate_camera ip_stream_node</code></pre>

<h3>Calibrate Camera</h3>
<pre><code>ros2 run get_calibrate_camera calibrate_camera</code></pre>

<h2>Development</h2>
<h3>Requirements</h3>
<ul>
    <li>Python (&gt;=3.8)</li>
    <li>ROS 2 Foxy (or higher)</li>
    <li>Additional Python libraries specified in the <code>package.xml</code></li>
</ul>

<h3>Build the Package</h3>
<pre><code>colcon build</code></pre>

<h3>Test the Package</h3>
![image](https://github.com/oscarpoudel/get_calibrate_camera/blob/main/images/sample_image.png)
<h2>Maintainer</h2>
<p><strong>Oscar Poudel</strong><br>
<a href="mailto:opoudel27@gmail.com">opoudel27@gmail.com</a></p>

<h2>License</h2>
<p>This package is licensed under <strong>[TODO: Specify License]</strong>.</p>

<h2>Contributing</h2>
<p>Contributions are welcome! Please follow the standard GitHub workflow:</p>
<ol>
    <li>Fork the repository.</li>
    <li>Create a new branch for your feature/bug fix.</li>
    <li>Submit a pull request with a clear description of your changes.</li>
</ol>
