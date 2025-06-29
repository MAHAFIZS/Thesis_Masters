using System.Windows.Media.TextFormatting;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.CompilerServices;
namespace iM
{
    partial class cpParadigmMenuOPT3
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }
        private void InitializeComponent()
        {
            this.smParadigmMenu = new iM.ScopeMonitor();
            this.state = new System.Windows.Forms.Label();
            this.motion_activated = new System.Windows.Forms.Label();
            this.output = new System.Windows.Forms.Label();
            this.orientation = new System.Windows.Forms.Label();
            this.axis_info = new System.Windows.Forms.Label();
            this.button1 = new System.Windows.Forms.Button();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            this.SuspendLayout();
            // 
            // smParadigmMenu
            // 
            this.smParadigmMenu.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.smParadigmMenu.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.smParadigmMenu.Location = new System.Drawing.Point(0, 0);
            this.smParadigmMenu.Name = "smParadigmMenu";
            this.smParadigmMenu.Size = new System.Drawing.Size(311, 268);
            this.smParadigmMenu.TabIndex = 0;
            // 
            // state
            // 
            this.state.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.state.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.state.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.state.Location = new System.Drawing.Point(360, 4);
            this.state.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.state.Name = "state";
            this.state.Size = new System.Drawing.Size(870, 61);
            this.state.TabIndex = 0;
            this.state.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // motion_activated
            // 
            this.motion_activated.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.motion_activated.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.motion_activated.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.motion_activated.Location = new System.Drawing.Point(0, 65);
            this.motion_activated.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.motion_activated.Name = "motion_activated";
            this.motion_activated.Size = new System.Drawing.Size(777, 61);
            this.motion_activated.TabIndex = 0;
            this.motion_activated.Text = "motion_activated";
            this.motion_activated.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // output
            // 
            this.output.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.output.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.output.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.output.Location = new System.Drawing.Point(780, 701);
            this.output.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.output.Name = "output";
            this.output.Size = new System.Drawing.Size(761, 54);
            this.output.TabIndex = 0;
            this.output.Text = "output";
            this.output.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // orientation
            // 
            this.orientation.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.orientation.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.orientation.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.orientation.Location = new System.Drawing.Point(16, 701);
            this.orientation.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.orientation.Name = "orientation";
            this.orientation.Size = new System.Drawing.Size(761, 54);
            this.orientation.TabIndex = 0;
            this.orientation.Text = "orientation";
            this.orientation.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // axis_info
            // 
            this.axis_info.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.axis_info.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.axis_info.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.axis_info.Location = new System.Drawing.Point(0, 100);
            this.axis_info.Name = "axis_info";
            this.axis_info.Size = new System.Drawing.Size(600, 100);
            this.axis_info.TabIndex = 0;
            this.axis_info.Text = "axis_info";
            this.axis_info.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            // 
            // button1
            // 
            this.button1.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button1.Location = new System.Drawing.Point(11, 4);
            this.button1.Margin = new System.Windows.Forms.Padding(4);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(229, 50);
            this.button1.TabIndex = 7;
            this.button1.Text = "Start";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // pictureBox1
            // 
            this.pictureBox1.Location = new System.Drawing.Point(1193, 130);
            this.pictureBox1.Margin = new System.Windows.Forms.Padding(4);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(348, 567);
            this.pictureBox1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pictureBox1.TabIndex = 8;
            this.pictureBox1.TabStop = false;
            // 
            // pictureBox2
            // 
            this.pictureBox2.Location = new System.Drawing.Point(0, 130);
            this.pictureBox2.Margin = new System.Windows.Forms.Padding(4);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(1185, 567);
            this.pictureBox2.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pictureBox2.TabIndex = 9;
            this.pictureBox2.TabStop = false;
            // 
            // cpParadigmMenuOPT3
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1557, 764);
            this.Controls.Add(this.state);
            this.Controls.Add(this.motion_activated);
            this.Controls.Add(this.orientation);
            this.Controls.Add(this.output);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.pictureBox1);
            this.Controls.Add(this.pictureBox2);
            this.Margin = new System.Windows.Forms.Padding(4);
            this.Name = "cpParadigmMenuOPT3";
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            this.ResumeLayout(false);

        }

        private iM.ScopeMonitor smParadigmMenu;
        private System.Windows.Forms.Label state;
        private System.Windows.Forms.Label motion_activated;
        private System.Windows.Forms.Label orientation;
        private System.Windows.Forms.Label output;
        private System.Windows.Forms.Label axis_info;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.PictureBox pictureBox2;
    }
}
